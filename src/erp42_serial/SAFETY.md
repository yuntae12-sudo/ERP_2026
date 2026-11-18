# ERP42 Serial safety changes — offline implementation

The modified node has **not been deployed to the running vehicle**. Hardware
motion, braking, E-Stop effectiveness and release interlocks remain unverified.
Existing workspace build/devel and the running node must remain untouched.

## Interface and packet contract

DriveCmd/ModeCmd/CmdControl/SerialFeedBack definitions and existing topic names
are unchanged. KPH and Deg remain raw commands: no additional scaling/sign change.
Command frames remain 14 bytes: STX, mode, E-Stop, gear, big-endian speed and steer,
brake at byte 10, alive at byte 11, CR LF. Feedback remains 18 bytes with the existing
little-endian fields. Floating-point division preserves raw speed 1 = 0.1 km/h
and signed steer / 71, including the -32768 boundary. Encoder remains a signed raw
count; no distance conversion is assumed. `/command` reports a fully written
packet, including policy overrides and alive. It is not a vehicle acknowledgement.

## Safety state machine

- **Locked:** startup, E-Stop, timeout, invalid command, feedback mismatch or I/O
  failure. Discard speed/steer; output configured stop brake/E-Stop. Keep the
  previously applied gear/mode (startup: Manual and explicitly configured gear).
- **Releasing:** only an explicit private `~rearm` Trigger service request may enter
  this state. Require healthy writes, fresh Auto/release ModeCmd, fresh zero
  speed/steer DriveCmd, and stationary raw speed/unchanged encoder observations.
  Request release/mode/gear with speed/steer zero and stop brake. Require a newer
  feedback alive counter acknowledging all three, with stationary hold, before
  arming. Motion commands received before acknowledgement are discarded.
- **Armed:** only a new DriveCmd after arming can produce motion. Brake commands
  different from the configured release value override speed and steer to zero.
  Gear changes lock motion and require another stationary rearm. A confirmed
  Manual policy enables the staged Manual path below; otherwise a Manual request
  faults Locked without assuming a vehicle interlock.
- **ManualTransition:** discard motion immediately. Keep the applied Auto mode,
  release E-Stop and send zero speed/steer + stop brake. Reset the stationary
  observation window on entry. Only fresh zero raw speed/unchanged encoder held
  for stationary_hold permits requesting Manual with the same gear. Require a
  full successful Manual packet write and a newer advancing-alive Manual/release
  feedback while stationary before completing. New DriveCmd cannot move here.
- **ManualLocked:** Manual acknowledgement has been observed; continue zero
  speed/steer + stop brake, keep Manual and released E-Stop. No ROS motion is
  accepted. Auto ModeCmd returns to Locked and fresh zero DriveCmd plus explicit
  rearm and subsequent fresh Auto feedback/new Drive are required again.

A clear EStop field or renewed command stream never automatically rearms.
Callbacks check for expiry before recording a new timestamp. Drive, mode and
feedback freshness use `steady_clock`; identical alive counters do not refresh
feedback liveness. E-Stop assertions remain actionable on duplicate counters.
The raw encoder/speed stationary criterion is a software precondition, **not a
proof of physical standstill**. Local receipt times cannot detect a sender's
stale queued command; the ROS messages have no timestamp/request ID. The rearm
service is a workflow gate, not authenticated operator authorization.

## Default gates and configuration

`protocol_confirmed=false` prevents the node from opening any port.
`allow_hardware_serial=false` restricts the transport to resolved `/dev/pts/`
paths. The existing launch explicitly retains both false defaults. No hardware
parameter profile is supplied. The following values must be verified for the
specific vehicle before a configuration can be accepted:

| Parameter | Default | Meaning |
|---|---:|---|
| stationary_only | true | Reject nonzero speed/steer; force both to zero at output and packet boundary |
| test_deadline_sec | -1 | Mandatory positive finite session deadline; missing/invalid policy refuses Serial |
| protocol_confirmed | false | Operator has verified the protocol and stop policy |
| allow_hardware_serial | false | Additional explicit hardware-access gate |
| brake_min / brake_max | -1 / -1 | Confirmed raw brake bounds, not physical units |
| brake_release / stop_brake | -1 / -1 | Confirmed distinct release/stop raw values |
| estop_assert / estop_release | -1 / -1 | Confirmed distinct raw E-Stop values |
| allowed_gears / initial_gear | empty / -1 | Confirmed gear values and startup gear |
| speed_limit_raw | -1 | Application command upper bound, in existing raw units |
| steer_limit_raw | 0 | Steering disabled; confirmed raw bound needed to enable |
| drive_timeout / mode_timeout / feedback_timeout | 0.25 seconds each | Software freshness policy, not a vehicle watchdog specification |
| stationary_hold | 0.3 seconds | Minimum observation hold, at least two advancing-alive samples |
| io_timeout_ms | 20 ms | Partial frame/write bound; accepted node range 1–100 ms |
| manual_transition_confirmed | false | Separate vehicle-specific Manual/brake interlock gate; Mock explicit opt-in |
| manual_transition_timeout | 1.0 seconds | Overall stop-and-ack deadline; repeated Manual requests never extend it |

All sent brake values are checked against the supplied bounds. No maximum brake
force, actual minimum speed, brake unit, gear-1 meaning or E-Stop interlock is
assumed. Test configuration brake=37, range=0..255, EStop=1/0 and gears=0/1/2 are
**synthetic fixtures**, not a deployable vehicle configuration.
The configured stop policy must work both in startup Manual and in Auto; code
cannot establish that physical property. No automatic switch to Manual is used
as a replacement for braking during a fault.

### Brake contract and Manual output contract

Control.cpp remains unchanged: it clamps the calculated brake to 1..200 and then
overwrites DriveCmd.brake with 1. With a configured release value of 0, that is
a braking command and must suppress speed/steer. The driver now diagnoses a
nonzero speed/steer request combined with a non-release brake, showing both the
input brake and configured release value. It does not translate legacy 1 to 0,
guess release=1, infer brake force, or treat brake as E-Stop. A valid ordinary
brake request remains a brake request without asserting E-Stop automatically.
Bounds validation and byte-10 delivery are unchanged. Missing/unconfirmed brake
policy keeps configuration invalid and Open refuses Serial before device access.

| State/phase | MorA | EStop | Brake | Speed/Steer |
|---|---|---|---|---|
| Locked | Previously applied (startup 0) | configured assert | stop_brake | 0/0 |
| Releasing | Requested Auto=1 | configured release | stop_brake | 0/0 |
| Armed | Applied Auto=1 | configured release | validated DriveCmd brake | new command, except non-release brake forces 0/0 |
| ManualTransition: observing stop | Applied Auto=1 | configured release | stop_brake | 0/0 |
| ManualTransition: awaiting acknowledgement | Manual=0 | configured release | stop_brake | 0/0 |
| ManualLocked | Manual=0 | configured release | stop_brake | 0/0 |

Manual is requested via the existing ModeCmd.MorA=0, distinct from the rearm
service. It is accepted only from Armed, with confirmed Manual policy, unchanged
gear, healthy transport and fresh feedback reporting released E-Stop. Releasing
or fault-Locked cannot use a Manual request as a shortcut around a fault/E-Stop.
ModeCmd.EStop assertion always has priority. All three 0.25s watchdogs still apply
to ManualTransition and ManualLocked. Invalid input, link failure, E-Stop, gear
change or interruption of the Manual transition goes to Locked with the configured
fault output; its MorA may be 1 or 0 depending on which phase was reached.
Repeated same-gear released Manual requests are idempotent. No old motion is
cached or reapplied in either Manual state or after Auto re-entry.
The caller must keep Mode/Drive inputs fresh throughout the transition (prefer
zero Drive and same-gear Manual Mode). A competing Auto publisher interrupts
Manual return and faults Locked; this change does not add command arbitration.

The release E-Stop + stop brake combination used during normal Manual return
requires vehicle validation; it is intentionally gated off in the live launch.
ManualLocked proves a software handshake, not physical brake holding force or
that manual operator motion is impossible. Manual feedback alone before a
successful Manual write cannot complete the transition. If the request times
out before changing MorA, faults retain Auto; after changing MorA, faults retain
Manual. No automatic rollback or serial framing recovery is assumed.

User-reported manual observations include brake=22 at EStop=0 standstill,
brake=200 with EStop=1 stop, and brake=200 during EStop=0 ordinary braking.
These are feedback observations, not verified Auto command values; brake=200
is therefore not treated as synonymous with E-Stop. No deployable hardware
profile has been added, and all three live launch confirmation/access flags stay
false.

## Transport and fault limits

Nonblocking reads perform at most one read per loop, retaining partial frames.
Missing data does not block watchdog evaluation or suppress a stop write. Partial
frame expiry and malformed headers/terminators latch a fault. No arbitrary
header scanning or transport resynchronization is introduced. Write retries use
only the unwritten suffix of the same 14-byte frame with a steady deadline.
EINTR/EAGAIN retries are bounded. A failed write poisons transmission: no full
frame replay or automatic reopening. Recovery requires a deliberate transport
recovery procedure verified against the vehicle protocol; this is not implemented
as an automatic action. Complete link loss can prevent any stop packet delivery.
Graceful shutdown performs one bounded best-effort stop write. SIGKILL, power loss
and process failure cannot guarantee that write. A vehicle-side watchdog and an
independent physical stop mechanism must be verified before vehicle testing.
The loop target is 20 ms; write latency, callback scheduling and OS scheduling add
reaction time. A software timeout is not a hard real-time response bound.

## Isolated reproduction

From the test branch, run only:

```bash
python3 -B src/erp42_serial/test/run_offline.py --sanitize
```

The runner creates a new `/tmp/erp42-safety-*` workspace, reads the two source
packages through symlinks, generates messages/build outputs there, and uses only
`/opt/ros/noetic` as its Catkin underlay. It does not launch ROS nodes, connect to
ROS Master, publish ROS commands or open a vehicle device. PTY tests use `openpty`;
mock system calls inject short writes, EAGAIN/EINTR, deadlines and read faults.
Logs, XML and JSON stay under that isolated directory. The script can be slow
while building; existing workspace build/devel are never build destinations.

SAFE-006 uses the separate `MotionWindow(1.0)` scenario utility with virtual
monotonic time, refreshing normal watchdog inputs throughout. The generic Serial
node does not impose a global one-second drive limit. SAFE-015 records speed and
encoder samples together in test XML. SAFE-018 PASS means graceful-stop packet
construction and the **absence** of a stop packet after a mock child's SIGKILL
were verified; physical stop is not PASS. ROS callback/service integration was
subsequently exercised by the isolated ROS/PTY verification recorded below.

## Before hardware testing

Verify brake bounds/units/release and holding force, gear mappings/startup gear,
E-Stop application and release sequence, alive watchdog semantics, frame recovery,
minimum controllable speed, speed accuracy and encoder behavior. Supply reviewed
hardware policy only after those checks. Start with stationary communication and
physical-stop checks under independent operator control. Future 0.1 km/h / one
second motion is not authorized by these offline results; reverse and steering
require separate limits and validation.

## Verified result — 2026-10-10 (Asia/Seoul)

Final isolated workspace: `/tmp/erp42-safety-r_wd5g32`.
Node and test build PASS; 36/36 tests PASS both normally and with ASan+UBSan.
SAFE-001 through SAFE-020 all passed their software/packet assertions. Sixteen
additional regressions include PTY/controller one-second round trip, framing,
encoder hold, callback freshness, release acknowledgement, duplicate-alive E-Stop,
steering gate, default device gate and integer signed boundaries.

| ID | Result | Evidence in safety_test.cpp |
|---|---|---|
| SAFE-001 | PASS | Startup motion blocked; unconfirmed config cannot rearm |
| SAFE-002 | PASS | Drive-only input cannot rearm |
| SAFE-003 | PASS | Mode-only input cannot rearm |
| SAFE-004 | PASS | Both Mode/Drive arrival orders remain locked |
| SAFE-005 | PASS | Fresh raw speed 1 produces the expected 14-byte packet |
| SAFE-006 | PASS | Separate one-second window stops with continuing fresh inputs |
| SAFE-007 | PASS | Drive timeout locks; resumed motion alone is discarded |
| SAFE-008 | PASS | Mode timeout locks |
| SAFE-009 | PASS | E-Stop discards speed/steer |
| SAFE-010 | PASS | E-Stop clear does not restart |
| SAFE-011 | PASS | Write failure locks; success alone does not rearm |
| SAFE-012 | PASS | Read failure locks and blocks rearm |
| SAFE-013 | PASS | Feedback stale blocks motion |
| SAFE-014 | PASS | Brake byte preserved; brake/speed conflict outputs zero motion |
| SAFE-015 | PASS | Raw speed 1 -> 0.1; speed/encoder samples recorded in XML |
| SAFE-016 | PASS | Signed steering/scaling and signed integer boundaries |
| SAFE-017 | PASS | Suffix retries; failed partial frame is never replayed |
| SAFE-018 | PASS (limit verified) | Graceful-stop packet; SIGKILL produces no stop packet |
| SAFE-019 | PASS | Gear change discards speed and retains previously applied gear |
| SAFE-020 | PASS | Explicit rearm, release acknowledgement and new DriveCmd required |

`offline-report.json`, `safety-results.xml`, `sanitized-results.xml` and
`environment-verification.json` in that isolated workspace are the recorded
results for the earlier offline run. Physical stop remains NOT TESTED; ROS
adapter runtime was subsequently verified in the isolated run below.
All 7,672 protected files in the original build/devel and full backup matched
content hashes, modes and modification times. Original Serial PID 5082/start time
remained unchanged. Four message MD5s matched original definitions and the
independently regenerated messages. No Git commit was made. Master remains
`852d1bbfe090b5786597b7dfe21084f6b7b8dae9`; backup and test HEAD remain
`bce6d56eb86ef60b78a2cc6483cfde9525229633`. Changes are confined to erp42_serial.

## Isolated ROS integration — 2026-10-10 (Asia/Seoul)

The added `test/run_ros_integration.py` launches a separate rosmaster and the
current node through mandatory bubblewrap isolation. No SafetyControl methods
are called by this Python test. Inputs are published through actual ROS topics;
rearm is an actual std_srvs/Trigger call. Every test-node namespace has a new
device mount and PID/proc namespace, zero capabilities, read-only host files,
and only host PTYs exposed as serial devices. The test checks the running child
device descriptors (Mock PTY and ROS's /dev/null only). Actual vehicle device
nodes and original process descriptors are hidden. Master-port isolation alone
is deliberately insufficient. If the isolation probe fails, ROS is not started.
Signals are sent only after checking the created test child's executable and
process start time, guarding against PID reuse. Only created test processes are
cleaned up. No live launch file is used.

Reproduction from the test branch (Mock only):

```bash
python3 -B src/erp42_serial/test/run_offline.py --sanitize
# Substitute the fresh /tmp workspace printed by the preceding command:
python3 -B src/erp42_serial/test/run_ros_integration.py --workspace /tmp/erp42-safety-REPLACE
```

Final build/regression workspace: `/tmp/erp42-safety-rzn600ej`.
Final integration directory: `/tmp/erp42-ros-integration-bmfrvvf7`.
Test Master URI: `http://127.0.0.1:38937` (terminated at test completion).
Test serial paths were fresh `/dev/pts/9` through `/dev/pts/13`; these names are
ephemeral, not persistent hardware configuration. Exact per-run paths, child
PIDs, mount namespaces, configuration and environment are in the report JSON.
All 24 integration cases passed. The first attempt exposed a test-wrapper
signal-target error (SIGINT to bubblewrap instead of its child); the harness was
corrected and the full suite rerun. Another fixture assertion was corrected to
allow ROS's harmless /dev/null FD alongside the PTY. Production C++/header/launch
code was not changed in this integration task, nor were any safety gates relaxed.
Watchdog timeouts remain 0.25s and the stationary hold uses the default 0.3s.

| ID | Result | ROS/PTY evidence |
|---|---|---|
| ROS-001 | PASS | Fresh empty Master; only test driver and test Serial node registered |
| ROS-002 | PASS | OS-isolated Mock PTY; unconfirmed configuration opens/writes nothing |
| ROS-003 | PASS | All four regenerated message MD5s match original build |
| ROS-004 | PASS | New DriveCmd after approval produces raw speed 1 |
| ROS-005 | PASS | ModeCmd Auto/release/gear reflected in captured command |
| ROS-006 | PASS | All 62 completed sampled command messages match exact 14-byte frames |
| ROS-007 | PASS | All 61 completed sampled feedback messages match Mock 18-byte frames |
| ROS-008 | PASS | Startup speed request remains zero/brake/EStop Locked |
| ROS-009 | PASS | Trigger denies absent/nonzero inputs; allows fresh zero/Auto/stationary inputs; new Drive required |
| ROS-010 | PASS | Drive stream alone stopped: lock in 0.254s |
| ROS-011 | PASS | Mode stream alone stopped: lock in 0.263s |
| ROS-012 | PASS | Feedback alone stopped: lock in 0.264s; rearm denied while stale |
| ROS-013 | PASS | EStop ModeCmd discards active motion |
| ROS-014 | PASS | EStop release does not restart continuing speed command |
| ROS-015 | PASS | Gear change discards speed and retains previously applied gear pending rearm |
| ROS-016 | PASS | Synthetic brake 73 appears at byte 10; speed/steer zero |
| ROS-017 | PASS | Raw speed 1 -> Mock feedback 0.1; raw steer 71 -> feedback 1 degree |
| ROS-018 | PASS | Mock scenario sends explicit zero/brake after 1s; observed packet by 1.052s |
| ROS-019 | PASS | SIGINT final captured packet requests zero/brake/EStop; best effort only |
| ROS-020 | PASS | SIGKILL/restart with continuing speed stream starts Locked, no cached approval |
| ROS-021 | PASS | PTY hangup produces POSIX read/poll failure and logged latch |
| ROS-022 | PASS | Same hangup produces write failure; command publication stops; Trigger denies recovery |
| ROS-023 | PASS | Damaged header and split frames beyond 20ms deadline not published as valid feedback |
| ROS-024 | PASS | Resumed streams alone stay Locked; explicit rearm and new Drive required |

`integration-report.json` records results, subprocess environment, binary hash,
namespace/FD proofs, synthetic configuration and process cleanup.
`captures.json` records monotonic-time topic fields, Trigger responses and every
captured/sent Serial frame. `node-*.log` record adapter safety states/errors.
`environment-verification.json` records the protected-environment comparison.
Normal and ASan/UBSan core/transport regressions are 36/36 PASS each, with XML
and logs in the build/regression workspace. The ROS executable itself was not
instrumented by those sanitizer checks.

Protocol values used here (stop brake 37, extra brake 73, EStop 1/0, gears 0/1/2)
are synthetic fixture choices, never verified vehicle specifications. ROS-018
is a scenario publisher explicitly commanding stop after one second; the
generic node still has no global one-second motion timer. No false latch was
observed during that one-second stream with 30ms command updates; this is not
a scheduling/stress guarantee. ROS-021/022 use one shared link-loss fault, not
independent injected read-only/write-only syscall faults. ROS-023 verifies
rejection of late fragments; timely valid fragmentation under every scheduling
condition is not established. The 20ms frame deadline and 20ms node period can
reject a frame that needs another loop to assemble; this is conservative locking,
not a proven reliable fragmented-stream transport or automatic resynchronizer.

Physical braking, EStop effects, gear meanings/interlocks, encoder stationarity
accuracy, minimum controllable speed, controller alive/link-loss behavior and
hardware recovery remain HARDWARE_UNVERIFIED. SIGKILL or cable loss cannot
guarantee a stop packet or physical stop; independent vehicle safety mechanisms
must be established before any hardware run. This test does not authorize one.
No commit/push was made; all current safety changes remain uncommitted.

## Brake contract / Manual return verification — 2026-10-10 (Asia/Seoul)

Latest build and core/transport regression workspace:
`/tmp/erp42-safety-z4sny757`.
Latest isolated ROS integration directory:
`/tmp/erp42-ros-integration-au9g9po8`.
Test Master: `http://127.0.0.1:58901`, stopped after completion.

All 59 offline cases PASS normally and with ASan/UBSan: the original 36 named
cases were retained and passed, four CONTRACT cases and ten MANUAL cases passed,
and nine new regressions passed. The regressions cover disabled Manual policy,
post-write/new-feedback acknowledgement, Serial read/write failure, callback
orders, duplicate alive + E-Stop priority, independent Manual Drive/Mode
watchdogs and invalid transition deadlines. A first unit run exposed a fixture
using an older fake timestamp after Drive; the timestamp was corrected, with no
production safety relaxation, before the final full normal/sanitized run.

| New ID | Result | Assertion |
|---|---|---|
| CONTRACT-001 | PASS | Non-release brake with requested motion suppresses speed and diagnoses release mismatch |
| CONTRACT-002 | PASS | Missing release/stop/min/max brake policy invalidates configuration |
| CONTRACT-003 | PASS | Legacy brake=1 with release=0 remains brake=1 in packet, zero motion, released E-Stop |
| CONTRACT-004 | PASS | Brake plus asserted E-Stop produces fault stop |
| MANUAL-001 | PASS | Auto-to-Manual request immediately discards motion |
| MANUAL-002 | PASS | Moving feedback cannot request/complete Manual early |
| MANUAL-003 | PASS | Held stationary observations then full Manual write and new Manual feedback enter ManualLocked |
| MANUAL-004 | PASS | Drive during transition never applies motion |
| MANUAL-005 | PASS | Repeated Manual requests cannot postpone overall timeout; faults Locked |
| MANUAL-006 | PASS | Manual exit requires new Auto/zero inputs, explicit rearm and new post-arm Drive |
| MANUAL-007 | PASS | E-Stop latch cannot be bypassed by a Manual request |
| MANUAL-008 | PASS | Stale feedback prevents completion and faults Locked |
| MANUAL-009 | PASS | Old motion never returns after Manual or a subsequent fault |
| MANUAL-010 | PASS | Repeated valid Manual requests are idempotent |

All original ROS-001..024 cases PASS again, plus:

| New ROS ID | Result | Topic/PTY/service evidence |
|---|---|---|
| ROS-025 | PASS | ModeCmd Manual -> logged ManualTransition -> stopped Auto packet -> Manual packet -> new Manual feedback -> logged ManualLocked |
| ROS-026 | PASS | Continuing nonzero Drive stream remains zero/brake during and after transition |
| ROS-027 | PASS | Auto request alone remains Locked; Trigger denies nonzero Drive |
| ROS-028 | PASS | Fresh zero Drive, Trigger approval, fresh Auto feedback and new Drive allow Auto re-entry |
| ROS-029 | PASS | ROS brake=1/release=0 conflict preserves byte10=1, zeros motion and logs exact mismatch |

`integration-report.json`, `captures.json`, `node-*.log` and
`environment-verification.json` in the latest integration directory record
results, actual Topic/Trigger/packet data, state transitions and protection
checks. Original 36-case identity was checked against the preceding test XML.
All four generated message MD5s still match the original workspace; 14/18-byte
packet contracts remain unchanged. Sanitizers cover the C++ core/transport test
binary, not the ROS executable itself.

Protection verification: 9,055 build/devel/backup file and directory entries
matched hashes, modes and mtimes with no additions/removals. Control.cpp hash,
master/backup/test branch heads and Remote remained unchanged. Original Serial
PID 5082/start tick 28147 was the same process. All created test processes were
cleaned up. No hardware Serial access or live ROS control publication occurred.
Production changes this task are confined to safety_control.h/.cpp, the Serial
ROS adapter and the disabled Manual launch gate; test and documentation updates
remain inside erp42_serial. No build output was installed into the live workspace.

This completes software verification only. Vehicle brake release/stop values and
bounds, E-Stop effect/release, mode-switch interlocks, stationary evidence quality,
controller watchdog and independent physical stop behavior remain unverified.
The next hardware stage is a separately authorized stationary validation with
independent physical protection and reviewed parameter values. No motion trial
or live-node replacement is authorized or performed by this work.


## Stationary-only and session deadline implementation — 2026-10-10

`stationary_only=true` is the production default. Positive speed or any signed
nonzero steer is rejected, diagnosed and faults Locked before input freshness is
updated or motion is stored. Every OutputDrive and final Packet encoding forces
speed/steer to zero while enabled, including startup, Releasing, Armed, Manual
states, faults and graceful shutdown. This does not infer physical standstill:
gear/mode changes, brake release, manual operator input and link loss remain
vehicle-specific hazards. Existing gear/rearm/Manual/EStop contracts remain.

Stationary policy accepts an explicitly configured speed_limit_raw=0; the default
-1 remains invalid. Non-stationary policy requires a positive raw bound, and motion
regressions explicitly opt out only in the isolated Mock configuration. Protocol
and hardware-access defaults remain false; unconfirmed Brake/EStop/gear policies
remain invalid. No Mock value has been installed as a hardware default.

`test_deadline_sec=-1` is fail-closed. Every otherwise valid configuration requires
an explicit positive finite limit. The epoch is SafetyControl construction, before
Serial Open, not first Drive, Auto transition or rearm. The core accepts injected
steady time for unit tests; the ROS adapter uses steady_clock::now. Startup waits,
faults, Manual transitions and rearm attempts all consume the same session budget.
Deadline is checked before the Locked early-return and every timestamped entry
(Drive/Mode/feedback/rearm/Packet); at elapsed >= limit it irrevocably locks this
instance, discards motion/zero-approval/input freshness and refuses every rearm.
Deadline has priority over coincident watchdog/EStop/transport diagnostics and
keeps configured stop brake/assert EStop with the previously applied mode/gear.
A defensive injected time before the epoch also expires the session; a backwards
wall clock or ROS /clock never changes the steady epoch. There is no reset service.

The current loop continues best-effort stop requests and read-only feedback
publication after expiry; expiry does not close the port or force Manual. Write
failure still poisons transmission and can prevent delivery of any stop request.
The timer is evaluated on the node loop/callbacks, not an independent hardware
interrupt; blocked scheduling, SIGKILL, power loss and link loss remain outside its
guarantees. A process restart creates a new epoch but starts Locked and needs fresh
inputs, stationary feedback and explicit rearm; it is not authenticated operator
approval. Hardware operators must repeat the external approval procedure. Live
ROS parameters are loaded at construction and cannot renew the running session.

| Timing policy | Meaning |
|---|---|
| Drive/Mode/Feedback 0.25s | Each input's freshness, independent of session deadline |
| Manual transition 1.0s | Stop/Manual-write/new-feedback handshake deadline |
| Stationary hold 0.3s | Fresh stationary samples held before mode/rearm decisions |
| Test deadline explicit positive seconds | Whole driver session from construction; never extended |
| MotionWindow 1.0s in Mock scenario | Separate future motion duration policy, not a production global motion timer |

The legacy stationary plan's statements that speed_limit_raw=0 is invalid and
that no global session timer exists describe the earlier source. They are
superseded by this implementation, not by new hardware evidence. Deployment is
still BLOCKED pending vehicle protocol, independent physical stop and reviewed
startup/Manual policies. Current launch explicitly sets stationary_only=true,
test_deadline_sec=-1 and all hardware confirmation/access flags false.

The offline runner now requires bubblewrap device/PID isolation before building
and runs C++ tests in a new /dev and private PTY instance, hiding vehicle devices
and original process descriptors. ROS integration retains its separate Master,
new device/PID namespaces, no capabilities and only Mock host PTYs. Core DEADLINE
009 proves steady-clock API dependency; actual ROS /clock jumps are tested through
the isolated ROS adapter, not claimed from a pure unit test.


Latest verification: `/tmp/erp42-safety-k0sv5qjy`: 85/85 normal and ASan/UBSan,
including all original 59 cases and 26 new cases. Isolated ROS directory
`/tmp/erp42-ros-integration-q7qf0l9q`: 39/39, including all original 29 and ten new
ROS/PTY cases. Test Master http://127.0.0.1:42867 was terminated; test child cleanup
verified. The generated node hash is
21958bd525dd0f73a253a9a9744b2f7c61fd7439b8118cd042d48859be211e2c.
Protection verified 9,055 unchanged build/devel/backup entries and the same live
Serial PID5082/start28147, binary, parameters, graph, branch heads, remote and index.
See [stationary/deadline report](docs/STATIONARY_DEADLINE_REPORT.md) for individual
IDs, evidence, initial fixture/isolation/log buffering failures and their fixes.
No hardware test, deployment, live control publication or commit was performed.
