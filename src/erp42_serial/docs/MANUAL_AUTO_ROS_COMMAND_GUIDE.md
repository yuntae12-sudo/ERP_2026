# ERP42 Manual ↔ Auto ROS 터미널 관측 안내

2026-10-10 KST / **실차 전환 BLOCKED**.
아래 실행 가능한 명령은 상태 조회·구독·새 파일에 기록만 한다. 이번 Codex 작업에서는 bag/관측 노드를 실차에서 실행하지 않았다.
DriveCmd/ModeCmd 발행·rearm·새 노드 launch·기존 노드 종료 명령은 포함하지 않는다.
미확인 hardware 계약을 채운 실차 실행 명령은 제공하지 않는다.

계획과 근거: [MANUAL_AUTO_TRANSITION_TEST_PLAN.md](MANUAL_AUTO_TRANSITION_TEST_PLAN.md).
현재 기존 driver PID5082는 stationary/deadline/rearm 구현을 사용하지 않는다.
기존 Serial 실행 터미널에서 Ctrl+C를 누르면 안 된다. 아래 별도 진단 터미널의 조회·bag process만 종료할 수 있다.

## PHASE A — 터미널 1: 환경 및 발행자 확인

새 Ubuntu 터미널에서 원본 생성 메시지를 사용한다. 환경 설정은 이 터미널에만 적용된다:

```bash
export PYTHONDONTWRITEBYTECODE=1
source /home/autonav/ERP_2024/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
printenv ROS_MASTER_URI
rosnode list
rosnode info /erp42_serial/erp42_serial_node
rosparam get /erp42_serial
rostopic info /erp42_serial/drive
rostopic info /erp42_serial/mode
rostopic info /erp42_serial/feedback
rosservice list
```

기대: node 한 개, 현재 drive/mode Publisher 없음, serial_port=/dev/ttyUSB2 및 baud115200.
현재 `~rearm`은 목록에 없다. 향후 candidate에서는 service 존재와 node 바이너리/config도 함께 확인해야 한다.
service 이름만으로 새 기능이 맞다고 확정하지 않는다. Parameter를 true로 바꾸는 방식으로 기존 바이너리를 안전 드라이버로 만들 수 없다.

실제 node PID와 exe를 읽기 전용으로 조회한다. hard-coded 5082를 다음 날 재사용하지 않는다:

```bash
python3 -B - <<'READ_NODE'
import os, pathlib, hashlib, xmlrpc.client
master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
caller='/erp42_manual_auto_readonly'
reply=master.lookupNode(caller,'/erp42_serial/erp42_serial_node')
assert reply[0]==1, reply
pid_reply=xmlrpc.client.ServerProxy(reply[2]).getPid(caller)
assert pid_reply[0]==1, pid_reply
pid=pid_reply[2]
p=pathlib.Path('/proc')/str(pid)
print('Master:',os.environ['ROS_MASTER_URI'],'PID:',pid)
print('exe:',os.readlink(p/'exe'))
print('start_tick:',(p/'stat').read_text().split()[21])
print('argv:',(p/'cmdline').read_bytes().replace(b'\x00',b' ').decode())
print('exe_sha256:',hashlib.sha256((p/'exe').read_bytes()).hexdigest())
READ_NODE
```

현재 기대 exe=`/home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node`,
SHA=`2c444c16402e3ca8e45497389aae4ef08ae2e92b31b2dbed329b21e4f353bbc3`.
이 hash와 source snapshot을 함께 사용하고, 빌드 provenance 없는 완전 source/binary 동등성을 주장하지 않는다.

포트 점유 조회: **fd 링크만 읽고 장치 또는 fd 자체를 open하지 않는다**.

```bash
python3 -B - <<'READ_OWNERS'
import os,pathlib
port='/dev/ttyUSB2'
owners=[]
for directory in pathlib.Path('/proc').glob('[0-9]*/fd'):
    try:
        entries=list(directory.iterdir())
    except PermissionError:
        print('접근 불가 PID:',directory.parent.name,'— 전체 점유 여부 미확인')
        continue
    except FileNotFoundError:
        continue
    for f in entries:
        try:
            if os.readlink(f)==port:
                owners.append((int(directory.parent.name),f.name))
        except OSError:
            pass
print('port:',port,'owners:',owners)
READ_OWNERS
```

현재 기대는 `(5082,'10')` 하나다. 장치 경로가 달라졌다면 rosparam 조회값으로 읽기 전용 script의 port만 바꾼다.
권한 때문에 검사 못한 프로세스가 있으면 “독점 점유 증명 완료”로 취급하지 않는다.
`cat /dev/ttyUSB2`, `stty -F`, pyserial, `/proc/PID/fd/10` open 등 직접 접근은 금지다.

## PHASE A/B — 터미널 2: 전체 Feedback 관측

터미널1과 같은 source/export를 한 뒤 실행:

```bash
rostopic echo /erp42_serial/feedback
```

초기 기준은 **MorA0 + 실제 Manual + 실제 정지**다. speed0·encoder 안정·alive 변화·EStop·Brake·Gear를 함께 본다.
현재 driver의 speed 정수 나눗셈은 0.1km/h 등을 0으로 표시할 수 있어 ROS speed0만으로 정지 증명 불가다.
encoder raw count는 거리나 바퀴 회전 해상도가 미확인이다. alive 증가는 수신 활동이며 차량 link-loss watchdog 증거가 아니다.

별도 작은 터미널에서 모드만 보고 싶으면:

```bash
rostopic echo /erp42_serial/feedback/MorA
```

기대 표시 순서는 향후 승인된 시험에서 `0 → 1 → 0`이다. 현재는 관측만 하고 이 변화를 유도하지 않는다.
command.MorA1보다 feedback.MorA1을 확인해야 한다. feedback0만으로 Manual handshake·물리 복귀 완료를 확정하지 않는다.
관측을 끝낼 때 해당 echo 터미널에서만 Ctrl+C를 누른다.

## PHASE B — 터미널 3: Bag 및 사건 기록

이 기록 명령은 제어 발행 없이 subscribe한다. 파일·로그·subscriber 연결 및 디스크/CPU 부하는 생긴다.
새 관측 경로만 사용하고 기존 backup/build/devel에 저장하지 않는다.

```bash
export PYTHONDONTWRITEBYTECODE=1
source /home/autonav/ERP_2024/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
OBS_DIR=$(mktemp -d /tmp/erp42-mode-observation-XXXXXX)
printf '%s\n' "$OBS_DIR"
df -h "$OBS_DIR"
rosparam dump "$OBS_DIR/erp42_serial.yaml" /erp42_serial
rosbag record -O "$OBS_DIR/manual-auto-observation.bag" \
  /erp42_serial/mode /erp42_serial/drive \
  /erp42_serial/command /erp42_serial/feedback /rosout /rosout_agg
```

drive/mode Publisher가 현재 없으면 해당 Topic 메시지가 기록되지 않는 것이 정상이다.
recorder는 구독자가 되며 자체 /rosout 로그를 발행할 수 있지만 drive/mode 발행은 하면 안 된다.
Topic 메시지에는 Header timestamp가 없으므로 bag 시각은 ROS 수신 시각이며 MCU 측정 시각이 아니다.
ROS/벽시계/steady time을 구분하고 operator 실제 조작 시각을 별도 적는다. 서비스 request/response는 bag Topic만으로 기록되지 않는다.

다른 메모 터미널에서 위 경로를 그대로 지정한 뒤, 사건마다 실제 설명을 붙인다:

```bash
# 앞서 출력된 새 관측 디렉터리를 직접 입력한 뒤 사용한다.
# OBS_DIR=/tmp/erp42-mode-observation-실제출력값
TZ=Asia/Seoul date --iso-8601=ns
```

기록 내용: operator·안전 담당자, physical Manual/정지 확인, EStop/Brake/gear 관측 조건,
시험 단계·조작 시각·기대값·실제값·중단 원인. 이번 관측에서는 조작을 유도하지 않는다.
향후 rearm이 승인되면 요청자/대상Master/node/요청시각/empty Trigger 요청/success/message를 별도 저장해야 한다.

## PHASE C — Manual → Auto: 현재 BLOCKED

**실행할 실차 ROS 발행/rearm 명령 없음.** 아래 확인이 끝나기 전까지 terminal4 제어 발행자를 만들지 않는다.

- command brake_release/stop_brake/bounds, EStop assert/release와 해제·startup 조합 확인.
- physical selector 우선순위, 허용 gear 및 정지 인터록, Manual 반환/실패시 독립 복귀 수단 확인.
- independent stop, 바퀴/차량 이동 방지, 운용자/안전 담당자 확보.
- 후보 바이너리와 reviewed profile, stationary=true, explicit positive Test Deadline, single-owner, single controlled publisher 확인.
- old Control의 고정 Auto/brake1 발행자를 병행하지 않으며 단일 발행자도 queued stale 입력을 배제한다고 가정하지 않음.

모두 확인된 다음 별도 승인된 절차:

1. startup Locked·feedback Manual0·실제 정지를 확인한다.
2. 단일 승인 발행자가 validated zero Drive(속도0/조향0/확인된 Brake)와 Auto Mode(확인된 release EStop/같은 gear)를 계속 갱신한다.
3. healthy link와 stationary hold 후 명시 rearm 결과를 확인한다. 거부되면 이유를 기록하고 무조건 재시도하지 않는다.
4. success 뒤 Releasing의 command1과 새 feedback1/gear/release 및 Armed 로그를 구분한다.
5. 전체 과정에서 physical 정지와 packet0/0을 유지한다. 예상외 mode/EStop/gear/encoder/움직임이면 즉시 중단·물리 보호 유지.

기본 Drive/Mode/feedback Watchdog 각각.25초다. hold.3초 동안 일회성 `rostopic pub -1`로 끝내면 timeout이 날 수 있다.
향후 갱신 주기는 승인된 운영 profile와 scheduling 측정으로 정한다. Mock30ms는 실차 보장이 아니다.
Auto ACK 전용 timer가 없으므로 fresh Manual feedback가 계속 오면 Test Deadline까지 Releasing일 수 있다.
정상 기대값과 운용자 ACK 중단 시간을 따로 정해야 한다. 미확인 실차 timeout 값을 지금 제시하지 않는다.

## PHASE D — Auto → Manual: 현재 BLOCKED

**실행할 실차 Mode 발행 명령 없음.** Physical Manual 복귀 수단과 manual_transition_confirmed의 차량 근거를 먼저 확보한다.

조건 충족 후의 승인 절차:

1. Armed, fresh feedback, released EStop, 같은 gear, 독립 정지를 확인한다.
2. zero Drive·same-gear Manual Mode를 계속 갱신한다. 입력 EStop assertion은 정상 Manual 반환을 대신하지 않는다.
3. Auto/zero/stop 출력과 요청 이후 stationary hold를 확인한다.
4. full Manual packet write 후 더 새로운 advancing-alive feedback0, released EStop, encoder 안정, ManualLocked 로그를 확인한다.
5. command0만 있고 feedback1이면 미완료다. timeout1초, watchdog, EStop, deadline이면 Locked fault이며 정상 복귀로 표시하지 않는다.
6. 이전 nonzero Drive 또는 Auto 재요청이 새 승인을 대신할 수 없으며, stationary 모드에서는 모든 비영 입력이 거부된다.

Manual 요청을 Locked 또는 Releasing에서 보내 Auto/Brake/EStop fault를 우회하려고 하지 않는다.
시나리오가 실패하면 이전 적용 Auto가 유지될 수 있으므로 독립 physical Manual·정지 수단이 반드시 필요하다.

## PHASE E — 종료 및 복구

향후 승인된 trial에서만:

- fresh feedback MorA0 + speed0/encoder 안정 + actual physical Manual/정지 + ManualLocked까지 확인.
- 시험 입력 종료 후 .25초 Watchdog로 **Locked, 적용 Manual0, assert EStop/stop Brake/0/0**으로 바뀔 수 있다.
  이 최종 stop 조합의 실제 Manual 효과를 미리 검증해야 한다. “발행자 종료 뒤에도 release EStop/ManualLocked 유지”를 기대하지 않는다.
- candidate 종료 뒤 새 feedback를 기대하지 않는다. 마지막 stop write는 best effort다. vehicle controller response는 UNKNOWN이다.
- bag 터미널의 Ctrl+C는 recorder만 종료한다. Serial 터미널 Ctrl+C와 구분한다.
- bag 종료 후 새 evidence 경로에서 아래 읽기 전용 확인:

```bash
rosbag info "$OBS_DIR/manual-auto-observation.bag"
sha256sum "$OBS_DIR/manual-auto-observation.bag" "$OBS_DIR/erp42_serial.yaml"
rostopic info /erp42_serial/drive
rostopic info /erp42_serial/mode
```

failure시 physical 정지 유지→증거 저장→별도 승인된 후보 정상 종료→port owner0→old exe/params/messages 복원 검토.
현재 운용 노드를 자동으로 kill/restart하는 명령은 제공하지 않는다. 롤백 뒤 Auto를 바로 재개하지 않는다.
세부 [Rollback 계획](STATIONARY_VALIDATION_ROLLBACK_PLAN.md)과 원본 [환경 기록](LIVE_ENVIRONMENT.md)을 따른다.

## Mock 재현용 명령 — 실차 실행 명령이 아님

브랜치 `test/erp42-low-speed-drive`에서 아래 전용 harness만 사용한다. 실제 launch는 실행하지 않는다.

```bash
python3 -B src/erp42_serial/test/run_offline.py --sanitize
# runner가 출력한 새로운 /tmp workspace를 아래에 지정한다.
python3 -B src/erp42_serial/test/run_ros_integration.py --workspace /tmp/erp42-safety-REPLACE
```

harness가 독립 Master를 생성하고 OS device/PID 격리를 검증한 후 Mock PTY에만 Topic/Trigger를 보낸다.
live MASTER11311을 대상으로 직접 pub/call하는 테스트 명령을 작성하지 않는다.
synthetic brake37/release0/EStop1/0/gears0,1,2는 Mock 선택값이며 실차 값으로 복사 금지다.


최종 Mock 검증: offline/sanitizer 각각85/85, ROS48/48 PASS.
실제 stage별 feedback와 failure cases는 [전환 계획 결과](MANUAL_AUTO_TRANSITION_TEST_PLAN.md)의 마지막 절,
환경 보호는 [manual_auto_verification.json](manual_auto_verification.json) 참조.
실차 판정은 여전히 **BLOCKED**다.
