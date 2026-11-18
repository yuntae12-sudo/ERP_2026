# Stationary-Only / Test Deadline 구현·검증 결과

2026-10-10 KST. 이번 범위는 소스 변경 및 격리 Mock 검증이다. **실차 적용은 BLOCKED**이며 현재 Serial 노드는 교체하지 않았다.
상세 기계 판독 결과: [stationary_deadline_results.json](stationary_deadline_results.json),
환경 보호: [stationary_deadline_environment_verification.json](stationary_deadline_environment_verification.json).
이전 [documentation_verification.json](documentation_verification.json)은 앞선 문서화 작업의 역사적 기록으로 유지했다.

## A. 구현

변경: `include/safety_control.h`, `src/safety_control.cpp`, `src/erp42_serial.cpp`,
`launch/erp42_serial_bridge.launch`, `test/safety_test.cpp`, `test/run_offline.py`,
`test/run_ros_integration.py`, `SAFETY.md` 및 erp42_serial/docs 문서/결과.
기존 Control, Total_msgs 정의, SerialPort 구현, 다른 패키지는 이번에 수정하지 않았다.

- stationary_only=true가 기본값이다. 비영 speed/steer는 freshness 갱신·명령 저장 전에 진단 후 Locked로 간다.
- OutputDrive와 Packet 최종 인코딩 직전에 0/0을 강제한다. Locked/Releasing/Armed/ManualTransition/ManualLocked와 종료 stop에 적용한다.
- stationary=true에 명시 speed_limit_raw=0 허용; false는 기존 양수 상한 요구. 정책 기본 -1은 자동 보완하지 않는다.
- test_deadline_sec 기본 -1, 누락·0·음수·NaN·무한대이면 invalid, Serial Open 거절. 정상 세션에는 명시 양수 필요.
- 기한은 SafetyControl 생성 시 steady_clock으로 시작해 Open/초기 대기/재승인/fault 시간을 모두 포함한다.
- 기한 도달 시 Locked, motion·zero 승인·입력 freshness 폐기. 새 Drive/Mode/feedback/rearm으로 연장·해제할 수 없다.
- 만료 시 기존 적용 mode/gear를 유지하고 설정 stop_brake/assert EStop/0/0 요청. Manual로 자동 변경하지 않는다.
- 별도 reset 서비스가 없다. 재시작은 새 epoch지만 초기 Locked이며 fresh inputs/정지 관측/명시 rearm이 다시 필요하다.
- 상태 enum, 메시지 및 Topic, 14-byte command/18-byte feedback 계약은 유지했다.
- 로그는 상태 값 또는 reason 변경 시 INFO로 남긴다. startup Locked에서 기한 만료한 경우에도 이유를 확인할 수 있다.

기한은 node loop에서 평가하므로 OS 정지/과도한 scheduling 지연/SIGKILL에 대한 hard real-time 보장이 아니다.
종료·케이블 단절·write poison 후에는 stop 전달을 보장할 수 없다. 재시작과 rearm은 인증된 안전 승인을 대신하지 않는다.
ROS params는 생성 때 읽으며 이후 parameter 변경은 실행 중 timer/policy를 재설정하지 않는다.

## B. 신규 오프라인 테스트

아래 요청 25개 모두 정상 및 ASan/UBSan에서 PASS. 추가 회귀 1개는 만료 이유가 transport fault/종료 처리로 사라지지 않음을 확인한다.

| ID | 결과 | XML testcase / 코드 위치 |
|---|---|---|
| STATIONARY-001 | PASS | `DefaultEnabled` (`test/safety_test.cpp:565`) |
| STATIONARY-002 | PASS | `PositiveSpeedRejected` (`test/safety_test.cpp:568`) |
| STATIONARY-003 | PASS | `NonzeroSteeringRejected` (`test/safety_test.cpp:573`) |
| STATIONARY-004 | PASS | `ZeroInputAccepted` (`test/safety_test.cpp:580`) |
| STATIONARY-005 | PASS | `ArmedZero` (`test/safety_test.cpp:584`) |
| STATIONARY-006 | PASS | `ReleasingZero` (`test/safety_test.cpp:585`) |
| STATIONARY-007 | PASS | `ManualTransitionAndLockedZero` (`test/safety_test.cpp:590`) |
| STATIONARY-008 | PASS | `EStopZero` (`test/safety_test.cpp:597`) |
| STATIONARY-009 | PASS | `GearInterlock` (`test/safety_test.cpp:601`) |
| STATIONARY-010 | PASS | `RejectedMotionNeverRestored` (`test/safety_test.cpp:607`) |
| STATIONARY-011 | PASS | `PacketBoundaryAndShutdown` (`test/safety_test.cpp:613`) |
| STATIONARY-012 | PASS | `DisabledUsesPositiveBound` (`test/safety_test.cpp:618`) |
| STATIONARY-013 | PASS | `MissingOrContradictoryPolicyBlocked` (`test/safety_test.cpp:623`) |
| DEADLINE-001 | PASS | `SessionStartsAtConstruction` (`test/safety_test.cpp:645`) |
| DEADLINE-002 | PASS | `ExpirationLocksMotion` (`test/safety_test.cpp:648`) |
| DEADLINE-003 | PASS | `PostDeadlineDriveRejected` (`test/safety_test.cpp:652`) |
| DEADLINE-004 | PASS | `PostDeadlineModeRejected` (`test/safety_test.cpp:656`) |
| DEADLINE-005 | PASS | `PostDeadlineRearmDenied` (`test/safety_test.cpp:659`) |
| DEADLINE-006 | PASS | `RepeatedRearmNeverExtendsSession` (`test/safety_test.cpp:663`) |
| DEADLINE-007 | PASS | `WatchdogSimultaneousExpiration` (`test/safety_test.cpp:671`) |
| DEADLINE-008 | PASS | `EStopSimultaneousExpiration` (`test/safety_test.cpp:675`) |
| DEADLINE-009 | PASS | `NoRosOrWallClockDependency` (`test/safety_test.cpp:679`) |
| DEADLINE-010 | PASS | `NewSessionStartsLocked` (`test/safety_test.cpp:686`) |
| DEADLINE-012 | PASS | `MotionWindowIndependent` (`test/safety_test.cpp:698`) |
| DEADLINE-011 | PASS | `InvalidLimitsFailClosed` (`test/safety_test.cpp:692`) |

DEADLINE-009는 순수 코어가 steady_clock 입력만 갖는 점과 만료 래치의 비가역성을 검사한다.
실제 ROS /clock 및 /use_sim_time 변경은 ROS-037에서 별도로 검증했다. 시스템 시계를 직접 변경하지 않았다.
DEADLINE-010은 새로운 코어 인스턴스가 Locked로 시작함을 확인하고, 기존 ROS-020은 실제 process 재시작 시 이전 승인이 복원되지 않음을 확인한다.
하드웨어 시험 PASS를 뜻하지 않는다.

## C. 전체 회귀와 실행 근거

| 검사 | 최종 결과 | 근거 |
|---|---|---|
| 일반 offline | **85/85 PASS** | `/tmp/erp42-safety-k0sv5qjy/safety-results.xml`, `safety-tests.log` |
| ASan/UBSan | **85/85 PASS** | 같은 경로 `sanitized-results.xml`, `sanitized-tests.log` |
| 이전 offline 59개 | 전부 유지·PASS | 이전 `/tmp/erp42-safety-z4sny757/safety-results.xml`와 suite/name 집합 비교, 누락0 |
| 격리 ROS | **39/39 PASS** | `/tmp/erp42-ros-integration-q7qf0l9q/integration-report.json` |
| 이전 ROS-001~029 | 전부 유지·PASS | 원래 ID29개 포함 확인 |
| 메시지 MD5 | 원본4개 일치 | ROS-003, 원본 build/devel 보호 해시 |
| command/feedback | 14/18 byte 형식 유지 | ROS-006/007 전체 completed sample 대응 및 captures.json |

ROS Master `http://127.0.0.1:42867`은 독립적으로 생성 후 종료했다. 실제 Master11311에는 시험 입력/서비스/파라미터를 보내지 않았다.
각 노드는 bubblewrap의 새 /dev 및 PID/proc namespace, capabilities0, Mock PTY만 사용했다.
원본 PID의 fd와 실제 USB/ACM 장치는 해당 namespace에서 접근할 수 없음을 실행 전 검사했다.
Offline 테스트도 새 /dev와 private PTY instance에서 실행하며, OpenPTY 가능 여부까지 probe했다.

| 신규 ROS ID | 실제 Topic/서비스/패킷 근거 |
|---|---|
| ROS-030 | stationary=true에서 positive Drive KPH를 ROS로 발행, 거부 로그·Locked·모든 PTY packet의 bytes6~9=0 |
| ROS-031 | 음수 조향을 ROS로 발행, steer 진단과 bytes6~9=0 |
| ROS-032 | zero Drive/Auto Mode와 실제 Trigger, Releasing/Armed 로그 및 zero packet |
| ROS-033 | EStop, fresh-zero 재승인, Manual 전환/ACK 및 정상 종료 패킷 모두 zero |
| ROS-034 | 입력을 계속 갱신해 watchdog를 회피하면서 session 기한만으로 Locked |
| ROS-035 | 만료 후 실제 Trigger success=false 및 deadline 이유 |
| ROS-036 | 만료 후 새로운 Drive/Mode·gear 요청·Trigger도 이전 gear/Locked/zero 유지 |
| ROS-037 | /clock=1000→2→5000→0 및 /use_sim_time 변경, steady deadline 만료/Trigger 거부 |
| ROS-038 | 양수 test_deadline 누락이면 다른 Mock 정책이 유효해도 port open/write 없음 |
| ROS-039 | stationary Armed에서 비영 Drive를 보내 Locked, 새 zero/rearm 이후에도 zero 유지 |

`captures.json`: ROS command/feedback 수신 시각·모든 필드, 실제 Trigger 요청 대상/응답 및 모든 PTY command/feedback hex.
`node-*.log`: state0~4, reason, stationary violation, deadline expiration 및 transport fault.
`integration-report.json`: Master URI·프로세스 환경·binary hash·실제 child device FD·namespace·synthetic profile·cleanup.
ASan/UBSan은 코어/transport 테스트 바이너리를 검사했으며 ROS 실행 파일 전체를 sanitizer로 검사했다고 주장하지 않는다.

최종 node SHA-256:
`21958bd525dd0f73a253a9a9744b2f7c61fd7439b8118cd042d48859be211e2c`.
모션 회귀는 격리 profile에서만 stationary_only=false, deadline60초를 명시했다.
신규 stationary profile의 brake37/release0/EStop1/0/gears0,1,2는 합성값이다. 실차 설정 파일을 만들지 않았다.

초기 실패와 수정:

1. 첫 신규 C++ test 빌드에서 initializer_list 타입 충돌을 발견해 명시 typed vector로 수정했다.
2. Offline namespace에서 host /dev/pts를 공유하면 이 환경의 ptmx로 새 PTY를 만들 수 없었다.
   offline에는 private PTY instance를 사용하도록 고쳤다. 실제 장치 차단을 제거하지 않았다.
3. 첫 ROS run에서는 INFO log가 파일에 늦게 flush되어 Manual state3 판정이 실패했다.
   테스트 child stdout/stderr를 stdbuf로 line buffering하고 재승인 시 새 Armed 로그를 기다리도록 했다.
4. 이후 전체85 일반/sanitizer와 ROS39 최종 결과를 확인했다. Production 안전 조건은 완화하지 않았다.

재현 (격리 검증만; live launch를 사용하지 않음):

```bash
python3 -B src/erp42_serial/test/run_offline.py --sanitize
# 위 명령이 출력한 새 /tmp/erp42-safety-* 경로를 지정한다.
python3 -B src/erp42_serial/test/run_ros_integration.py --workspace /tmp/erp42-safety-REPLACE
```

/tmp 산출물은 재부팅/정리에 사라질 수 있다. 이 작업에서는 기존 backup에 새 결과를 덮어쓰지 않았다.

## D. 환경 보호

- 기존 Serial PID5082 및 시작 tick28147 동일, 기존 exe 해시 동일. 실제 port owner는 PID5082 fd10만 확인했다.
- build/devel 및 전체 backup의9055 항목에 content hash/mode/mtime 변화와 추가·누락0.
- 기존 master/backup/test branch HEAD, Remote, Git index 동일; commit/push 없음.
- Control.cpp 해시, 기존 live ROS params/graph 동일. 메시지 정의 및 생성 계약 유지.
- 실제 USB/ACM port open 및 live Drive/Mode 발행 없음. 기존 노드 정상 종료/재시작 없음.
- 생성한 Mock 노드/독립 Master는 정리했다. 검증 JSON의 success=true는 환경 보존 성공이며 하드웨어 PASS가 아니다.

## E. 남은 실차 위험

Brake command bounds/release/stop과 holding force, EStop 효과·해제 조건, startup Manual assert+stop 조합,
Auto/Manual 및 gear 인터록, encoder stationary 신뢰도, controller alive/link-loss watchdog와 최소 구동속도는 HARDWARE_UNVERIFIED다.
수동 feedback22/200과 EStop0/1 관측은 Auto command 계약을 증명하지 않는다.
stationary0/0은 실제 차량 무이동을 보장하지 않는다. 모드/기어/브레이크 해제 및 수동 입력이 영향을 줄 수 있다.

## F. 다음 단계

정지 전용 출력 강제와 session time limit의 소프트웨어 준비는 완료했다.
그러나 live launch는 protocol_confirmed=false, allow_hardware_serial=false, manual_transition_confirmed=false,
stationary_only=true, test_deadline_sec=-1을 유지해 실차 활성화를 차단한다.
독립 물리 정지 및 제동/모드 계약 확인, 검토된 positive session limit와 profile, 바이너리/현재 미커밋 소스/의존성 복구본,
운용자·안전 담당자 승인이 선행되어야 별도 정지 상태 검증으로 넘어갈 수 있다.
Test Deadline은 전체 세션 제한이며 1초 주행 제한으로 전용하지 않는다. 주행 MotionWindow는 별도 검증 작업이다.
