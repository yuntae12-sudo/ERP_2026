# ERP42 실차 정지 상태 검증 및 Rollback 실행계획서

후속 소프트웨어 갱신: Stationary-Only와 session deadline 구현 내용은 [../SAFETY.md](../SAFETY.md)의 최신 절을 참조한다. 아래 speed_limit_raw=0 및 timer 부재 설명은 작성 당시 소스의 기록이며 현재 구현에서는 해소됐다. 하드웨어 BLOCKED 판단은 유지한다.

2026-10-10 KST / 현재 판단: **실차 드라이버 전환 및 Auto 활성화 BLOCKED**.
이번 작업은 조회와 문서 추가만 수행했다. 아래 변경·빌드·기록·종료·실행 명령은 **향후 별도 승인 후 사용할 설계이며 이번에 실행하지 않았다**.
현재 차량 명령, Serial 직접 접근, 노드 교체, 하드웨어 시험을 승인하는 문서가 아니다.

현재 환경: [LIVE_ENVIRONMENT.md](LIVE_ENVIRONMENT.md), 원본 조회: [live_environment_snapshot.json](live_environment_snapshot.json).
보호 검증: [documentation_verification.json](documentation_verification.json).
근거는 현재 미커밋 코드, 기존 백업 코드, 기존 테스트 결과와 사용자 수동 관측으로 한정했다.
제조사 사양 또는 외부 문서에 의존하지 않는다. 코드에서 증명되지 않는 물리적 성질은 UNKNOWN / HARDWARE_UNVERIFIED다.

## A. 현재 환경과 적용 전 증거

- Master `http://localhost:11311`; 노드 `/erp42_serial/erp42_serial_node`; PID 5082, 시작 tick 28147.
- 실행 바이너리 `/home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node`.
- 원본과 전체 백업 바이너리 SHA-256 모두 `2c444c16402e3ca8e45497389aae4ef08ae2e92b31b2dbed329b21e4f353bbc3`.
- `/erp42_serial/erp42_serial_node` Parameter는 serial_port=`/dev/ttyUSB2`, serial_baudrate=`115200`뿐이다.
- 실제 포트 점유는 조회 범위에서 PID 5082 fd 10 한 개다. 실행 중인 바이너리는 기존 드라이버다.
- 현재 drive/mode Publisher는 없다. 이는 기존 드라이버의 명령 소거 또는 차량 제동을 증명하지 않는다.
- test 및 backup HEAD `bce6d56eb86ef60b78a2cc6483cfde9525229633`; master `852d1bbfe090b5786597b7dfe21084f6b7b8dae9`다.
- 미커밋 안전 소스는 backup SHA에 포함되지 않는다. HEAD만 기록하면 현재 구현을 재현할 수 없다.

### 기존 시험 증거 (다시 실행하지 않음)

| 증거 | 결과와 범위 |
|---|---|
| `/tmp/erp42-safety-z4sny757/offline-report.json`, `safety-results.xml`, `sanitized-results.xml` | 정상 및 ASan/UBSan 각각 59/59, 코어/transport 소프트웨어와 Mock/PTY 시험 |
| `/tmp/erp42-ros-integration-au9g9po8/integration-report.json` | 격리 Master/Mock PTY에서 29/29, `hardware_tested=false`, 생성 프로세스 정리 완료 |
| 같은 디렉터리 `captures.json`, `node-*.log` | Topic/서비스/14·18 byte 패킷과 상태 전이 |
| `src/erp42_serial/SAFETY.md:321-387` | 최신 계약/Manual 검증과 한계 |

격리 테스트 바이너리 해시는 `2c702a6a94d785afc809c5fa594055ba54e0e8a0f5b37258398e64fa61e1ee0e`다.
/tmp 산출물은 임시다. 미래 배포용 복사본, 소스 해시, 빌드 환경, 동적 라이브러리 의존성 및 설정 해시를 별도 보존해야 한다.
Mock PASS는 실제 제동·EStop·최소속도·차량 통신 watchdog PASS가 아니다.

### 현재 드라이버와 새 소스의 중요 차이

| 항목 | 기존 백업 코드 | 현재 미커밋 새 코드 |
|---|---|---|
| Brake | callback 및 packet byte10이 0 고정 | 유효한 입력 raw brake 전달; release와 다르면 속도·조향 0; fault에는 설정 stop_brake |
| EStop | ModeCmd를 직접 반영 | fault/EStop 래치, 해제·입력 재개만으로 재출발 안 함 |
| 수신 중단 | 마지막 명령 유지 가능; Read 실패 시 Write도 생략 | 각 입력 및 feedback timeout, 누락 feedback에도 stop write 시도 |
| Write | 성공/부분 전송 보장 없음 | 전체 전송 성공 후 command 발행; 실패 뒤 전송 차단, 자동 재연결 없음 |
| 초기 송신 | Manual0/Gear1/speed0/steer0/brake0/EStop0 코드 | Manual0/설정 initial_gear/speed0/steer0/stop_brake/assert EStop |
| Manual | 요청값 직접 반영 | Armed에서만 단계적 정지·Manual write·새 feedback ACK; 별도 확인 gate |
| speed feedback | 정수 나눗셈으로 저속 0 표기 가능 | raw/10.0, 소수 유지 |

근거: 백업 `src/erp42_serial/src/erp42_serial.cpp:16-20,45-71,126-153,157-175`, 백업 header:18-36;
현재 `src/erp42_serial/src/erp42_serial.cpp:48-118,120-135`, `src/safety_control.cpp:68-235`.
기존 코드의 소스와 실행 바이너리 전체 동등성은 빌드 provenance 부재로 UNKNOWN이다.
기존 header:70의 alive 멤버는 초기화가 명시되지 않아 재시작 직후 카운터 재현성도 가정하지 않는다.

## B. 새 드라이버 전환 계획

### 순서, 진행 조건 및 중단 조건

| 단계 | 작업 / 진행 조건 | 중단 조건 및 대응 |
|---|---|---|
| P0 현재 상태 보존 | 기존 PID/시작 tick/실행 파일/port/params/graph 재조회, 본 기록과 비교 | 상태가 달라지면 새 기준 기록부터 작성. 기존 프로세스 유지 |
| P1 복구 자료 확보 | 빈 새 디렉터리에 기존 exe·현재 설정·메시지·동적 라이브러리 정보·현재 미커밋 소스와 /tmp 시험 로그 보존, 해시 확인 | 기존 backup에 덮어쓰기, 공간 부족, 파일 변경, 해시 불일치면 중단 |
| P2 배포물 준비 | 새로운 workspace에서 현재 두 package의 고정 복사본을 빌드·격리 재검증, 원본 build/devel에 설치하지 않음 | 소스 manifest와 테스트 바이너리 불일치, 기존 환경 쓰기 발생이면 중단 |
| P3 하드웨어 정책 승인 | 독립 물리 정지, 바퀴 이동/회전 방지, Brake/EStop/초기 Manual/gear/모드 인터록 확인 및 두 담당자 승인 | 현재 **BLOCKED**. feedback의 22/200을 명령값으로 전용하거나 flag만 true로 변경해서 우회 금지 |
| P4 기존 노드 중지 | 위 조건 충족 후 기존 운용자가 해당 launcher만 정상 종료. Master는 유지 | 독립 정지 미확보, 노드/launcher 식별 불확실이면 종료 금지 |
| P5 중복 방지 | 기존 PID 소멸, Master 등록 해제, 장치 owner 0 확인 후 새 실행 허용 | owner 존재/권한 부족으로 확인 불가, respawn 발견 시 새 실행 금지 |
| P6 후보 실행 | 확인된 별도 profile와 해시 확인된 절대 exe, 기존 namespace/topic 유지 | 현재 **BLOCKED**. 초기 송신 자체가 실차 명령이다. unexpected mode/gear/EStop, encoder 변화, wheel 움직임, stale/frame/write fault이면 시험 중단 |
| P7 정지 상태 체크 | HW표 순서, 승인된 zero 입력만 사용. Auto/rearm 관련 항목은 독립 gate | 각 FAIL/UNKNOWN에는 rearm 재시도 금지, 물리 정지 후 rollback 또는 시스템 유지 |
| P8 종료 | Manual handshake와 물리 Manual·정지 확인 후 후보 정상 종료, 통신 소멸에도 정지 유지 | Manual 실패 시 “노드 종료하면 안전” 가정 금지. 물리 수단으로 확보 후 종료/복구 판단 |

안전 담당자는 물리 EStop과 접근 통제 및 즉시 중단 판단을 담당한다. 기존 운용자는 Manual 선택과 차량 실제 상태를 확인한다.
시험 운영자는 하나의 명령 발행자 및 로그만 관리한다. 실패 시 물리 정지를 먼저 확보하고 시험을 멈춘다.
ROS 정지 요청이 전달되지 않을 수 있으므로 통신이나 서비스 응답을 기다리며 계속 시험하지 않는다.
통신 단절 시 차량 자체 반응은 UNKNOWN이며, rollback 중 통신 공백에도 같은 문제가 있다.

### 향후 승인 후 준비 명령 예시 — 이번에는 미실행

아래 `RUN_DIR`은 새 디렉터리만 허용한다. 실제 작업 전 용량과 복사 중 변경 여부를 확인한다.
기존 전체 backup과 build/devel은 읽기 원본으로만 사용한다.

```bash
RUN_DIR=/home/autonav/erp42_stationary_run_APPROVED_UNIQUE_ID
test ! -e "$RUN_DIR" || exit 1
mkdir "$RUN_DIR"
mkdir "$RUN_DIR/rollback" "$RUN_DIR/evidence" "$RUN_DIR/candidate"
cp -a /home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node "$RUN_DIR/rollback/erp42_serial_node"
cp -a /home/autonav/ERP_2024_backup/src/erp42_serial/launch/erp42_serial_bridge.launch "$RUN_DIR/rollback/legacy_bridge.launch"
ROS_MASTER_URI=http://localhost:11311 rosparam dump "$RUN_DIR/rollback/erp42_serial.yaml" /erp42_serial
sha256sum "$RUN_DIR/rollback/erp42_serial_node"
ldd /home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node > "$RUN_DIR/rollback/ldd.txt"
cp -a /home/autonav/ERP_2024/devel/include/Total_msgs "$RUN_DIR/rollback/Total_msgs_headers"
cp -a /home/autonav/ERP_2024/devel/lib/python3/dist-packages/Total_msgs "$RUN_DIR/rollback/Total_msgs_python"
mkdir "$RUN_DIR/candidate/src"
cp -a /home/autonav/ERP_2024/src/erp42_serial "$RUN_DIR/candidate/src/"
cp -a /home/autonav/ERP_2024/src/Total_msgs "$RUN_DIR/candidate/src/"
cp -a /tmp/erp42-safety-z4sny757 "$RUN_DIR/evidence/offline_artifacts"
cp -a /tmp/erp42-ros-integration-au9g9po8 "$RUN_DIR/evidence/integration_artifacts"
```

/tmp workspace가 symlink를 포함하면 위 cp -a는 링크를 보존한다. 그래서 candidate/src는 별도로 실제 package를 복사했다.
복구 자료에는 현행 workspace 전체의 새 독립 snapshot 또는 의존 라이브러리의 복사와 해시도 확보한다.
`ldd.txt`는 목록일 뿐 라이브러리 백업이 아니다. 기존 전체 backup 외부의 `/opt/ros/noetic`/시스템 라이브러리는 별도 의존성이다.
정확한 build/devel 복원은 절대 경로가 담긴 setup 파일도 고려해야 한다. 다른 경로에 복사하고 setup을 source한다고 완전 재배치되지 않는다.

후보를 빌드하는 작업은 이 요청에서 금지되어 있다. 향후에는 깨끗한 별도 셸에서 `/opt/ros/noetic`만 underlay로 사용한다:

```bash
# 별도 승인 후에만. 원본 workspace에서는 실행하지 않는다.
env -i HOME=/home/autonav USER=autonav PATH=/usr/bin:/bin RUN_DIR="$RUN_DIR" \
  bash --noprofile --norc -c 'source /opt/ros/noetic/setup.bash; catkin_make -C "$RUN_DIR/candidate"'
```

소스/메시지 해시와 새 바이너리 해시를 기록하고 독립 ROS/PTY 격리 재검증 후 배포 후보로 지정한다.
현재 /tmp Mock 설정을 복사한 실차 profile는 작성하지 않는다.
실차 실행 명령은 **현재 제공 불가/BLOCKED**다. 나중에 승인된 profile가 있어야 아래 절차를 구체화할 수 있다:

1. 확인된 기존 launcher PID/시작 tick/명령을 다시 비교한다. 이 기록의 부모 PID는 5045지만 미래에 그대로 쓰지 않는다.
2. 승인된 해당 launcher만 정상 SIGINT 종료한다. `kill -INT <검증된_기존_launcher_PID>`는 문서용 자리표시자다.
3. 실제 serial owner가 0이고 기존 node 등록도 없어졌는지 읽기 전용 확인한다. 같은 node 이름으로 병행 실행하면 기존 노드를 종료시킬 수 있다.
4. namespace private params를 승인 profile로 적용하고 절대 candidate exe를 같은 __ns/__name으로 실행한다.
   이때 protocol_confirmed와 allow_hardware_serial 변경, 초기 EStop/Brake 송신에는 별도 하드웨어 승인이 필요하다.
5. candidate 한 개의 포트 owner, startup Locked 로그와 zero command, feedback 및 물리 정지를 확인한다. 기대 값과 다르면 rearm하지 않는다.

기본 차단은 `src/erp42_serial/src/erp42_serial.cpp:8-16,19-38,48-57` 및
`src/SerialPort/serial_port.cpp:33-39`에 있다. gate false 또는 설정 invalid이면 port open을 거절한다.
`manual_transition_confirmed=false`는 Manual 경로를 차단하지만 전체 설정/Auto를 차단하지 않는다.
실차에 필요한 전체 정책의 확인은 사람이 검토해야 하며 bool true 자체는 검증 증거가 아니다.

### ROS 제어 계약과 단계별 출력

| 인터페이스 | 계약 |
|---|---|
| drive: DriveCmd | `uint16 KPH`, `int16 Deg`, `uint8 brake` raw 입력. 추가 단위 변환 없음. bounds 불일치이면 lock. non-release brake와 움직임 입력이 함께 오면 진단 및 속도/조향0 |
| mode: ModeCmd | `uint8 MorA,EStop,Gear,alive`; code Manual0/Auto1, EStop/gear는 확인된 설정과 비교. alive 입력은 사용 안 함 |
| command: CmdControl | 실제 14 byte 전체 write 성공 후 해당 내용 발행. 차량 수락·실제 제동의 ACK가 아님. 실패하면 발행 및 이후 전송 중단 |
| feedback: SerialFeedBack | 노드가 자체 Serial read로 발행. raw speed/10.0, steer/71.0; encoder raw signed count, brake raw byte, alive 수신 카운터. 단위 계수는 코드 계약이며 실차 정확도 미확인 |
| private ~rearm | std_srvs/Trigger, `/erp42_serial/erp42_serial_node/rearm`; 현재 기존 노드에는 없음. Locked/유효 정책/healthy write/새 Auto-release Mode/새 zero Drive/정지 hold가 필요 |

메시지 정의 행: `src/Total_msgs/ERP/DriveCmd.msg:4-6`, `ModeCmd.msg:4-7` 및 동일 폴더 CmdControl/SerialFeedBack.
Adapter 행: `src/erp42_serial/src/erp42_serial.cpp:40-46,60-96,120-135`.
Protocol codec 행: `src/erp42_serial/src/safety_control.cpp:29-55`.

| 상태 | mode / EStop / Brake / motion |
|---|---|
| Locked 0 | 기존 적용 mode(초기 Manual0), assert, stop_brake, 0/0 |
| Releasing 1 | 요청 Auto1, release, stop_brake, 0/0; rearm 성공만으로 Armed가 아님 |
| Armed 2 | Auto1/release, 새 drive brake; 새 drive만 적용, non-release는 0/0 |
| ManualTransition 3 전반 | Auto1/release/stop_brake/0/0, 요청 후 정지 hold 관측 |
| ManualTransition 3 후반 | Manual0/release/stop_brake/0/0, full write 뒤 새 advancing-alive Manual feedback 대기 |
| ManualLocked 4 | Manual0/release/stop_brake/0/0; 물리 Manual 운용자의 움직임을 차단한다는 뜻은 아님 |

근거 `src/safety_control.cpp:96-129,131-199,202-235`, `SAFETY.md:98-127`.
Manual 요청은 Armed에서만 정상 전환을 시작한다. Locked에서 MorA0 요청만 보내도 안전 복귀가 된다고 가정하지 않는다.
모든 fault는 Locked로 가며 현재 적용 mode를 유지한다. Auto fault에서는 Auto가 유지될 수 있다.
정지 hold는 0.3초지만 Mode/Drive timeout은 0.25초이므로 transition 동안 동일 Manual/same gear와 zero Drive를 계속 갱신해야 한다.
예를 들어 향후 통제된 시험 발행 주기를 30ms로 계획하되 OS/통신 지연 측정 후 결정한다. Mock 30ms 성공은 실차 시간 보장이 아니다.
기존 Control.cpp는 Mode 뒤 Drive를 발행하지만 Topic 간 callback 순서는 보장되지 않는다. callback은 timeout을 먼저 평가한다.
새 드라이버에는 publisher 인증/중재가 없다. 단일 발행자라도 비정상 입력·queued stale 메시지를 모두 배제할 수 없다.
`Control.cpp:398,418-420`의 Auto1 및 brake1을 그대로 병행하면 Manual 복귀 또는 release 계약과 충돌할 수 있다.
이번 정지 시험에는 기존 Control/CAP 발행자를 시작하지 않는다. 경쟁 발행자를 실제로 만들어 HW-014를 시험하지 않는다.

## C. Rollback 실행계획

**롤백은 기존 동작 복구이며 안전성 보장이 아니다.** Brake 무시/Watchdog 부재를 복구하므로 자동 운전을 재개하지 않는다.

복구 trigger: 예상 mode/gear/EStop, 움직임/encoder 변화, frame/write 오류, feedback 중단, Manual timeout,
중복 publisher/port owner, 시험 시간 초과, 물리 안전 수단 불확실.

1. 안전 담당자가 독립 수단으로 물리 Manual·정지·바퀴 고정을 확보한다. 장치 종료나 통신 공백이 제동을 보장하지 않는다.
2. 운영자는 새 승인/입력 시험을 중단한다. fault 뒤 fresh zero나 rearm을 반복해 결과를 숨기지 않는다.
3. candidate 로그·bag·Parameter·현재 PID/start/exe/graph·실패 UTC/KST 시각을 새 run 디렉터리에 보존한다.
4. 후보가 연결된 상태라면 안전 확보 후 해당 후보 launcher/PID만 정상 종료한다. `kill -INT <검증된_후보_PID>`.
   graceful 종료 stop write는 최대한 시도할 뿐 성공/제동 보장이 없다. SIGKILL를 자동 fallback으로 사용하지 않는다.
   종료되지 않으면 물리 정지를 유지하고 복구 중단, owner가 남아 있는 동안 기존 노드를 시작하지 않는다.
5. 실제 owner 0, 후보 및 기존 node 등록 해제 확인. Master는 계속 유지한다.
6. 원본 old exe가 정상이라면 소스/원본 binary를 덮어쓰지 않고 그 절대 경로를 사용한다.
   손상되었으면 검증된 run/rollback exe 복사본을 그 자리에서 실행한다. 의존 라이브러리/원본 환경이 유효해야 한다.
7. 해당 namespace의 시험 Parameter만 제거하고 사전에 저장한 `/erp42_serial` snapshot을 복원한다.
   snapshot 외 기존 사용자가 있었다면 소유 범위를 재검토하고 namespace 전체 삭제도 하지 않는다.
8. 아래 SHA/MD5를 확인하고 old exe를 명시 namespace/name 및 port/baud로 실행한다. 기존 source launch를 재실행하는 방식은 피한다.
9. 다시 one owner/한 node/옛 feedback rate·값·alive/encoder·Manual/정지·명령 publisher0 확인한다.
   물리 또는 feedback 상태가 불일치하면 복구 성공으로 표시하지 않고 물리 안전 유지 및 운영 중단.
10. 시험 실패 로그를 보존하고 분석 승인 전 Auto 재개 금지. rollback 실패 시 git reset/restore/clean으로 해결하지 않는다.

### 미래 rollback 명령 — 미실행, 물리 정지 및 owner0 확인 후만

`RUN_DIR`은 P1에서 검증된 실제 새 디렉터리다. 현 상태에서 아래를 실행하면 실행 노드가 영향을 받으므로 금지한다.

```bash
# 후보 정상 종료 및 port owner 0 확인이 선행되어야 한다.
ROS_MASTER_URI=http://localhost:11311 rosparam delete /erp42_serial/erp42_serial_node
ROS_MASTER_URI=http://localhost:11311 rosparam load "$RUN_DIR/rollback/erp42_serial.yaml" /erp42_serial
sha256sum /home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node
# 기대 SHA: 2c444c16402e3ca8e45497389aae4ef08ae2e92b31b2dbed329b21e4f353bbc3
```

복구용 별도 셸에서 원본 devel 또는 검증된 동일 의존 환경을 사용한다. ROS Master를 새로 띄우지 않는다:

```bash
# 향후 승인 후, 실제 차량은 독립 수단으로 Manual·정지 유지.
source /home/autonav/ERP_2024/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_NAMESPACE=/erp42_serial
/home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node \
  __ns:=/erp42_serial __name:=erp42_serial_node \
  _serial_port:=/dev/ttyUSB2 _serial_baudrate:=115200
```

원본 exe를 사용할 수 없으면 마지막 절대 exe만 검증된 `$RUN_DIR/rollback/erp42_serial_node`로 바꾼다.
__log는 과거 경로를 재사용하지 않고 새 ROS 로그를 보존한다. 실행 방식은 기존 roslaunch와 다르지만 node namespace/name,
params, exe, messages 및 의존 환경을 명시한다. 감독 프로세스까지 동일하게 필요하면 별도 검토된 legacy launch를 사용한다.
백업 launch의 pkg lookup이 현재 환경을 참조하므로 backup launch만 지정하는 것은 backup exe 선택을 보장하지 않는다.

메시지 MD5 재확인 (입력 발행 없음, Python bytecode 생성 안 함):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B - <<'CHECK_MD5'
from Total_msgs.msg import DriveCmd, ModeCmd, CmdControl, SerialFeedBack
for cls in (DriveCmd, ModeCmd, CmdControl, SerialFeedBack):
    print(cls.__name__, cls._md5sum)
CHECK_MD5
ROS_MASTER_URI=http://localhost:11311 rosnode info /erp42_serial/erp42_serial_node
ROS_MASTER_URI=http://localhost:11311 rostopic info /erp42_serial/drive
ROS_MASTER_URI=http://localhost:11311 rostopic info /erp42_serial/mode
ROS_MASTER_URI=http://localhost:11311 rostopic echo -n 1 /erp42_serial/feedback
```

기대 MD5는 LIVE_ENVIRONMENT 표와 비교한다. feedback speed0만으로 정지 판정 금지: legacy 정수 나눗셈은 저속을 0으로 표시할 수 있다.
backup 브랜치는 소스 복구의 별도 checkout/새 디렉터리 검토에만 사용한다. 원본 미커밋 변경을 덮어쓰지 않는다.
전체 backup은 원본 overwrite 없이 새 경로에서 검토한다. snapshot의 절대 경로 의존성과 nmea 서브모듈 복원 불가를 함께 기록한다.

## D. HW-001~015 정지 상태 계획

이번 작업에서 모든 HW 항목은 **미실행**, 하드웨어 PASS를 부여하지 않는다.
공통 전제: 물리 Manual 확인, 허가된 바퀴 회전/차량 이동 방지, 독립 EStop의 동작·해제 후 유지 능력,
운용자/안전 담당자 배치, 전후방·주변 사람 접근 통제, 기록 준비, 예상 gear/encoder/feedback baseline.
GPS/IMU는 사용하지 않는다. physical condition은 feedback과 독립된 운용자 기록으로 남긴다.
초기 physical EStop 유지와 candidate assert/release의 조합, 조이스틱·차량 모드 selector의 우선순위도 미확인이다.

| ID | 향후 방법 / 필요한 승인 | 기대 증거와 중단 조건 | 현재 분류 |
|---|---|---|---|
| HW-001 | 기존 노드 그대로 feedback 구독, 물리 Manual 확인 | MorA0와 운용자 Manual 확인; 다른 mode면 정지 및 분석 | 수동 관측 가능 / 아직 이번 미실행 |
| HW-002 | 충분한 정지 baseline 구독, speed/encoder/time 기록 및 바퀴 관찰 | speed0, encoder 동일, fresh alive; legacy quantization·encoder 해상도 때문에 물리 관찰 필수 | 수동 관측 가능 / 물리 정확도 HARDWARE_UNVERIFIED |
| HW-003 | 검증된 release 명령 계약 및 independent holding 확보 후 별도 시험 | 해제 조건과 물리 제동 해제의 대응 확인; 임의 0/1/22 사용 금지. 어떠한 움직임에도 중단 | BLOCKED |
| HW-004 | 검증된 stop 명령·범위·Manual/Auto 적용 조건 확보 후 별도 시험 | feedback 및 독립 측정으로 실제 holding 확인; feedback200만으로 제동력 증명 금지 | BLOCKED |
| HW-005 | 기존 Manual에서 운용자의 물리 EStop 관측은 별도 승인; ROS EStop 시험은 추가 계약 승인 | 상태 변화와 실제 정지 유지. 사용자 보고 EStop1/brake200은 command 증명 아님 | Manual 관측 가능 / ROS 시험 BLOCKED |
| HW-006 | independent holding 유지, 운용자의 물리 해제 관측; Auto 해제는 별도 gate | 물리 재출발 없음, ROS 래치 유지 확인; zero 입력만으로 “움직임 허용 안 함”의 모든 경우는 증명 못 함 | Auto BLOCKED |
| HW-007 | 확인된 profile·초기 패킷·gear·Auto 요건 승인 후 zero inputs와 명시 rearm | 새 Auto feedback, startup Locked→Releasing→Armed; zero 유지. 모드 mismatch/움직임이면 중단 | BLOCKED |
| HW-008 | HW007 및 Manual gate 확인 후 same gear Manual 요청, zero Drive/Mode 갱신 유지 | 요청 후 stationary hold→full Manual write→새 advancing-alive MorA0→ManualLocked; timeout/fault시 중단 | BLOCKED |
| HW-009 | 승인된 zero-only Armed 시험에서 Drive 갱신만 중단, Mode/feedback 유지 | Drive timeout 로그+zero/stop/assert packet, 재개만으로 rearm 안 됨. 움직이는 차량 제동 성능은 증명하지 못함 | MOCK_VERIFIED / 실차 BLOCKED |
| HW-010 | 같은 정지 조건에서 Mode 갱신만 중단, zero Drive/feedback 유지 | Mode timeout 및 Locked 유지, 실제 정지 확인; 예상 이외 mode/EStop면 중단 | MOCK_VERIFIED / 실차 BLOCKED |
| HW-011 | 현재 live feedback publisher/Serial read를 중단하지 않음; 기존 격리 Mock 결과 사용 | 새 node는 ROS feedback를 구독하지 않으므로 rostopic 차단·remap으로 raw Serial loss를 흉내낼 수 없음. 별도 승인된 하드웨어 장애 방법 필요 | MOCK_VERIFIED / 실차 BLOCKED |
| HW-012 | 차량 측 링크 단절 동작 및 독립 정지를 먼저 증명한 뒤 별도 fault 계획 | fault log와 실제 차량 response; link loss 때 stop packet 미전달 가능. USB 탈착/프로세스 kill을 지금 수행하지 않음 | HARDWARE_UNVERIFIED / BLOCKED |
| HW-013 | 물리 정지 상태에서 사용자 수동 gear 관측; Auto gear는 매핑 승인 후 별도 검증 | gear 변화, encoder 안정; 새 driver의 gear 변경 lock/정지 rearm와 실제 차량 interlock 분리 | 코드/MOCK 검증, 하드웨어 BLOCKED |
| HW-014 | Master graph 반복 조회로 한 발행자만 허용, recorder는 drive/mode subscribe only | 시험 시 통제된 publisher1개, baseline/종료에는0개. 하나라도 외부 publisher면 중단. 실제 충돌 입력 금지 | 조회 가능 / 중재 기능 없음 |
| HW-015 | 승인된 HW008 완료 또는 운용자 물리 Manual 확인, 정지/holding/로그완료 후 종료 | MorA0+speed/encoder+물리 Manual/정지, 하나의 owner 또는 종료 뒤0. fault-Locked를 Manual 완료로 취급하지 않음 | 후보 실차 시험 BLOCKED |

HW003/004에서 정지 중 release/stop 전송을 하더라도 힘·정지거리·움직임 중 제동 성능을 검증했다고 표시하지 않는다.
holding 정량 판단 방법과 목표는 아직 UNKNOWN이다. 무리한 밀기/바퀴 돌리기를 확인 절차로 제시하지 않는다.
실제 Auto 제어로만 확인 가능한 사항: command 값의 차량 수락, startup stop 조합, Auto 전환·EStop 해제,
Auto에서의 브레이크 효과, Manual 인터록/정지 ACK, 실제 link-loss 대응.
무이동 시험으로 확인 불가한 사항: 0.1km/h 최소 구동 및 정확도, 운동 상태의 stopping distance/제동력, 후진 제동.

## E. 실차 파라미터 근거표

CODE_VERIFIED는 코드 동작, MOCK_VERIFIED는 합성 조건, MANUAL_OBSERVED는 사용자 또는 기록된 수동 feedback를 뜻한다.
같은 항목이라도 코드 계약과 물리 성질의 등급이 다르다. UNKNOWN은 현재 근거가 없는 성질이다.

| 파라미터 | 현재 소스 기본값 / 실제 기존 노드 값 | 근거 / 분류 / 실차 판단 |
|---|---|---|
| allow_hardware_serial | false / 기존 param 없음 | adapter:9-16, SerialPort:35-39 CODE_VERIFIED; symlink realpath 검사. 물리 접근 승인 BLOCKED |
| protocol_confirmed | false / 없음 | adapter:21,48-51, SafetyConfig::Valid:15-26 CODE_VERIFIED; 하드웨어 계약 확인 HARDWARE_UNVERIFIED |
| brake_release | -1 / 없음 | adapter:25, Drive:75-94 CODE_VERIFIED; Mock0 MOCK_VERIFIED; 실제 해제 raw UNKNOWN |
| stop_brake | -1 / 없음 | adapter:22, OutputDrive:227-231; Mock37 MOCK_VERIFIED; 사용자 feedback200 MANUAL_OBSERVED, 실제 명령·holding UNKNOWN |
| brake_min/max | -1/-1 / 없음 | Valid:16-19; Mock0..255는 byte 범위, 차량 유효 범위 UNKNOWN |
| estop_assert/release | -1/-1 / 없음 | adapter:26-27, Mode:103, OutputMode:224; Mock1/0. 사용자 Manual feedback1/0 MANUAL_OBSERVED; Auto command/해제 조건 HARDWARE_UNVERIFIED |
| manual_transition_confirmed | false / 없음 | adapter:36, Mode:105 CODE_VERIFIED; Mock true는 합성. 실제 interlock HARDWARE_UNVERIFIED |
| manual_transition_timeout | 1.0s / 없음 | header:55, Tick:177-178 CODE_VERIFIED 및 Mock 검증. 반복 요청 연장 안 됨; 실제 충분한 시간인지 UNKNOWN |
| drive/mode/feedback_timeout | 각각0.25s / 없음 | header:50-52, Tick:172-174 CODE_VERIFIED/MOCK_VERIFIED; 차량 watchdog 사양 아님 |
| stationary_hold | 0.3s / 없음 | header:53, Stationary:165-168; 최소2개 advancing-alive 및 raw speed0/encoder 동일. 센서 고장/물리 정지 신뢰도 UNKNOWN |
| speed_limit_raw | -1 / 없음 | Valid:22에서 positive 필수, Drive:78; Mock10. 실제 단위 계수/최소속도 및 안전 상한 HARDWARE_UNVERIFIED |
| steer_limit_raw | 0 / 없음 | header:48, Drive:80; code 조향 금지 기본값. Mock1000을 실차에 적용 금지 |
| allowed_gears/initial_gear | empty/-1 / 없음 | Valid:22-24, ctor:60. Mock0/1/2 및 initial1; legacy Gear0 D/2 R 사용 흔적은 실제 인터록 증거 아님 |
| serial_port | launch /dev/ttyUSB2 / 실제 /dev/ttyUSB2 | launch:7, /proc/5082/fd/10 CODE_VERIFIED·실제 조회. 포트 번호의 다음 재부팅 안정성 UNKNOWN |
| serial_baudrate | 115200 / 실제 param115200 | launch:8, node:15-18, SerialPort:57-61 CODE_VERIFIED; 장치 직접 검사 안 함 |
| io_timeout_ms | 20ms / 없음 | adapter:8-13, SerialPort:63-109 CODE_VERIFIED/MOCK_VERIFIED; frame 조각이 다음20ms loop로 넘어가면 reject 가능, 실차 신뢰도 미검증 |
| 최대 시험 시간 | 생산 node parameter 없음 | MotionWindow header:104-113/cpp:237-242는 별도 utility; ROS018의1s는 publisher 명시 stop, node global timer 아님 |

**정지 전용 profile의 현재 한계**: speed_limit_raw=0은 configuration invalid다.
positive 상한을 임의로1로 넣어 “정지 전용 안전”이라고 표시하지 않는다. zero 명령 계획은 입력 계획일 뿐 전체 node의 nonzero 금지를 강제하지 않는다.
전역 시험 deadline/zero-only 구동 봉쇄가 필요하면 별도 구현·격리 검증 작업이 필요하며 이번에 변경하지 않았다.

Brake 근거 상세: 사용자 수동 정지 EStop0/brake22, 물리 EStop 정지 EStop1/brake200, 일반 제동 EStop0/brake200
(`SAFETY.md:129-134`). 이는 MANUAL_OBSERVED **feedback**이며 release/stop **command** 계약으로 치환 불가.
기존 Control brake1(`Control.cpp:418`)은 LEGACY 코드 입력, 200은 Stop 목표 코드의 흔적(:77-116,352)이지만
실제 발행값과 다른 위치가 있다. Mock37/73, release0 및 byte0..255는 합성 fixture다.
차량 측 alive watchdog, 통신 중단 후 유지/제동/모드, gear1 의미, startup assert+stop의 효과는 UNKNOWN이다.

## F. 기록 계획과 최종 판단

### 향후 승인 후 ROS bag 기록 명령 (이번 미실행)

```bash
# bag 경로는 P1의 새 RUN_DIR 내부. 이것은 subscriber 기록이며 drive/mode를 발행하지 않는다.
ROS_MASTER_URI=http://localhost:11311 rosbag record \
  -O "$RUN_DIR/evidence/stationary.bag" \
  /erp42_serial/command /erp42_serial/feedback \
  /erp42_serial/drive /erp42_serial/mode /rosout /rosout_agg
```

bag는 node를 종료·재시작하지 않지만 subscriber/로그와 저장 부하는 생긴다. 공간·CPU와 기록 누락을 확인한다.
rosbag recorder의 /rosout 자체 로그는 있을 수 있으나 차량 제어 Topic 발행은 없어야 한다.
이 메시지들에는 Header timestamp가 없으므로 bag timestamp는 수신 시각이다. node 내부 steady_clock은 wall/ROS time과 다르다.
UTC/KST wallclock, 시스템/ROS 시계 설정, operator 조작시각, 이벤트 ID/상태/물리 관측/판정/담당자/실패 사유를 별도 기록한다.

- command: mode/EStop/Gear/KPH/Deg/brake/alive 및 수신시간. 전체 write 내용이지 차량 ACK 아님.
- feedback: mode/EStop/Gear/speed/steer/brake/encoder/alive 및 수신시간. 물리 관찰과 대조.
- drive/mode: 입력 raw 값·발행자 목록·수신시간. zero-only 준수와 timeout 입력 중단시각 확인.
- /rosout: node 상태0..4, timeout/fault/contract conflict, walltime. /rosout_agg와 중복 가능한 점 기록.
- rearm: 서비스는 topic bag만으로 request/response가 저장되지 않는다. 향후 승인된 각 호출의 시각,
  요청자·요청 내용(Trigger는 empty)·success/message·대상 Master/node·호출 전후 zero 입력 및 feedback를 별도 저장한다.
  현재 rearm 호출 명령은 제공하지 않는다. 승인 없는 호출을 유도하지 않기 위해 BLOCKED 단계에 둔다.
- 물리 운용자: joystick/Brake/EStop/Manual selector/gear 조작 전후 사건시각과 실제 차량 상태; 이번 조작 없음.

기존 시험 캡처 원본 파일은 수정하지 않는다. run evidence에 copy/hash로 보존한다.
시험 후 rosbag info와 connection metadata/Topic 타입/MD5를 확인하고 bag·log·config·binary·source manifest 해시를 함께 저장한다.
기존 Serial 드라이버의 command 필드는 실제 성공한 frame와 일치하는 증거가 없으므로 후보 command와 의미를 구분한다.

### 진행 가능 범위와 남은 최소 조건

지금 가능한 것은 현재 실행 환경을 유지한 수동 feedback 관측 계획, 소프트웨어 증거 검토 및 별도 배포물 준비 설계다.
현재 실차 node 교체·새 초기 packet·Auto/rearm·Brake/EStop 명령·HW003~013 실행은 **BLOCKED**다.

필요한 최소 조건:

1. 독립 물리 정지와 통신이 끊긴 상태의 정지 유지, 운용자·안전 담당자 및 무이동 환경 확인.
2. 실제 차량 command brake release/stop/bounds, EStop 적용/해제, startup Manual stop 조합과 모드 우선순위 확인.
3. Gear mapping/정지 인터록, Auto→Manual 및 physical selector 조건, 센서 stationary 신뢰도 확인.
4. 차량 통신 watchdog/실패 시 동작 확인. software watchdog로 차량 watchdog의 존재를 대체하지 않음.
5. 현재 미커밋 소스와 logs의 독립 보존, candidate build provenance, old exe/params/MD5/의존성 검증된 복구본 준비.
6. zero-only 강제/전역 시험 deadline의 현 한계를 검토하고 필요시 별도 소프트웨어 개선 및 격리 재검증.
7. 확인된 profile와 단계별 책임자 승인을 확보한 뒤, 별도 승인된 정지 시험부터 진행. 0.1km/h 주행은 그 뒤에도 별도 작업이다.

프로토콜 자료가 없다는 이유로 값을 추정해 gate를 해제하지 않는다. 기존 사용자 수동 피드백은 관측 가능한 근거로 남기되
Auto command 계약을 증명하기 전에는 실차 적용 완료/PASS로 표시하지 않는다.
