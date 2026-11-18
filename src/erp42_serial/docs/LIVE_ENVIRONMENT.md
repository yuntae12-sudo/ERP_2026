# ERP42 현재 실차 실행 환경 기록

작성일: 2026-10-10 KST. 상세 조회 시각 및 원본 응답은 [live_environment_snapshot.json](live_environment_snapshot.json) 참조.
이 문서는 읽기 전용 조회 결과다. 노드·포트·파라미터를 변경하거나 차량 명령을 발행하지 않았다.

## 현재 실행 상태

| 항목 | 확인 결과 / 증거 |
|---|---|
| Master | `http://localhost:11311`, 실행 프로세스 환경 및 Master XML-RPC 조회 |
| 노드 | `/erp42_serial/erp42_serial_node`, Master lookupNode 및 노드 XML-RPC getPid=5082 |
| PID / 시작 tick | `5082` / `28147`; `/proc/5082/stat` 필드 22, PID 재사용 판별에 필요 |
| 실행 파일 | `/home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node`, `/proc/5082/exe` |
| 작업 디렉터리 | `/home/autonav/.ros`, `/proc/5082/cwd` |
| 부모 | PID 5045, `/usr/bin/python3 /opt/ros/noetic/bin/roslaunch erp42_serial erp42_serial_bridge.launch` |
| 실제 노드 인수 | `__name:=erp42_serial_node __log:=/home/autonav/.ros/log/2a8c982c-c495-11f1-881b-512fa6748b34/erp42_serial-erp42_serial_node-1.log` |
| namespace | 프로세스 환경 `ROS_NAMESPACE=/erp42_serial` |
| 포트 / 설정 baud | `/dev/ttyUSB2` / `115200`, 아래 Parameter 및 포트 fd 조회 |
| 장치 점유 | `/proc/*/fd` 조회에서 PID 5082의 fd 10만 확인. 조회 권한 범위 내 결과이며 후속 실행 직전 재확인 필수 |
| 현재 브랜치 / HEAD | `test/erp42-low-speed-drive` / `bce6d56eb86ef60b78a2cc6483cfde9525229633` |
| master / backup | master=`852d1bbfe090b5786597b7dfe21084f6b7b8dae9`; backup=`bce6d56eb86ef60b78a2cc6483cfde9525229633` |
| Remote | `origin ssh://fs-admin@165.246.170.75:7751/home/fs-admin/ERP_2024.git` |

baud는 적용 Parameter에서 확인한 값이다. 장치를 열어 termios 또는 차량 UART 상태를 재측정하지 않았다.
프로세스 환경: `ROS_PACKAGE_PATH=/home/autonav/ERP_2024/src:/opt/ros/noetic/share`,
`CMAKE_PREFIX_PATH=/home/autonav/ERP_2024/devel:/opt/ros/noetic`,
`PYTHONPATH=/home/autonav/ERP_2024/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages`, `ROS_DISTRO=noetic`.

## 적용 Parameter와 그래프

Master에서 `/erp42_serial` 전체를 조회한 결과:

```yaml
erp42_serial_node:
  serial_port: /dev/ttyUSB2
  serial_baudrate: 115200
```

새 안전 파라미터가 존재하지 않는다. 현재 노드에 `~rearm` 서비스도 없다.

| Topic | 타입 | 현재 Publisher | 현재 Subscriber |
|---|---|---|---|
| `/erp42_serial/drive` | `Total_msgs/DriveCmd` | 없음 | Serial 노드 |
| `/erp42_serial/mode` | `Total_msgs/ModeCmd` | 없음 | Serial 노드 |
| `/erp42_serial/command` | `Total_msgs/CmdControl` | Serial 노드 | 없음 |
| `/erp42_serial/feedback` | `Total_msgs/SerialFeedBack` | Serial 노드 | 없음 |

`/rosout`는 Serial 노드가 발행하고 `/rosout` 노드가 구독하며, `/rosout_agg`는 `/rosout` 노드가 발행한다.
Master의 Topic 타입 목록에는 과거 카메라 Topic도 남아 있으나 활성 발행 여부는 getSystemState 기준이다.
명령 Publisher가 없다는 사실은 이전 명령이 지워졌다는 보장이 아니다. 기존 노드는 Watchdog가 없다.

## 바이너리 / 현재 소스 관계

실행 파일 SHA-256:

`2c444c16402e3ca8e45497389aae4ef08ae2e92b31b2dbed329b21e4f353bbc3`

전체 백업의 `devel/lib/erp42_serial/erp42_serial_node`도 동일하다.
`nm -C`에서 기존 `ERP42Serial::readUpdate()`, `writeUpdate(...)`, `DriveCallback(...)` 심볼을 확인했다.
새 `Rearm` 구현은 현재 소스 `src/erp42_serial/src/erp42_serial.cpp:132`에 있으나 실행 노드 서비스에는 없다.
따라서 현재 프로세스는 새 안전 구현이 배포된 프로세스가 아니다.
해시 일치만으로 바이너리가 어느 소스 커밋/컴파일 옵션으로 만들어졌는지 완전히 증명할 수는 없다.

현재 launch `src/erp42_serial/launch/erp42_serial_bridge.launch:7-12`는 port/baud와 세 가지 false gate를 포함한다.
전체 백업의 같은 launch:7-8은 port/baud만 포함한다. 부모 명령은 launch 이름을 증명하지만 시작 당시 파일 내용은
현재 파일과 다를 수 있다. 새 launch를 재실행해서 기존 실행 구성이 그대로 복원된다고 가정하지 않는다.
롤백은 아래 실행 바이너리, namespace, 두 Parameter 및 메시지 환경의 명시적 복원 기준으로 설계한다.

## 메시지 복원 기준

정의 위치: `src/Total_msgs/ERP/{DriveCmd,ModeCmd,CmdControl,SerialFeedBack}.msg`.
`src/Total_msgs/CMakeLists.txt`의 ERP DIRECTORY 설정으로 생성한다. 파일을 msg/로 이동할 필요가 없다.

| 메시지 | 원본 devel 생성 헤더의 MD5 / 행 |
|---|---|
| DriveCmd | `69a82c1e3ba067908d8fabb7e7d8716a`, `devel/include/Total_msgs/DriveCmd.h:137` |
| ModeCmd | `93db1f5d15225b4e964829a0cf6a3bdb`, `devel/include/Total_msgs/ModeCmd.h:143` |
| CmdControl | `17043bad9780a7db1065d8002634203b`, `devel/include/Total_msgs/CmdControl.h:161` |
| SerialFeedBack | `977be1b73fb4d99913310d68e9241255`, `devel/include/Total_msgs/SerialFeedBack.h:167` |

## 보존 자료와 한계

- 전체 백업 `/home/autonav/ERP_2024_backup`: 실행 파일, build/devel, 미추적·ignore 파일 포함.
- 초기 검증 자료: `/home/autonav/ERP_2024_backup.{manifest,verification,final-verification}.json`.
- 증분 bundle `/home/autonav/ERP_2024_backup.branches.bundle`: 기존 HEAD 객체를 전제로 한 증분이며 단독 전체 백업이 아니다.
- backup 커밋은 현재 미커밋 안전 구현을 포함하지 않는다. source snapshot JSON은 내용 해시 기록이며 소스 자체의 복사본이 아니다.
- `src/nmea_navsat_driver`는 삭제 상태 그대로다. 기존 gitlink 참조 객체가 없으므로 원래 서브모듈 내용 완전 복원 불가.
- 기존 실제 노드 로그는 계속 증가할 수 있다. 이 작업은 로그를 동결하거나 프로세스를 중단하지 않았다.

## 읽기 전용 재확인 명령 (제어 명령 없음)

```bash
ROS_MASTER_URI=http://localhost:11311 rosnode info /erp42_serial/erp42_serial_node
ROS_MASTER_URI=http://localhost:11311 rosparam get /erp42_serial
ROS_MASTER_URI=http://localhost:11311 rostopic info /erp42_serial/drive
ROS_MASTER_URI=http://localhost:11311 rostopic info /erp42_serial/mode
readlink /proc/5082/exe
tr '\0' ' ' < /proc/5082/cmdline
sha256sum /home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node
nm -C /home/autonav/ERP_2024/devel/lib/erp42_serial/erp42_serial_node | rg 'ERP42Serial::(Rearm|readUpdate|writeUpdate)'
git -C /home/autonav/ERP_2024 show-ref --heads
git -C /home/autonav/ERP_2024 remote -v
```

Port 점유 조회는 `/proc/*/fd`의 링크만 읽는다. 장치 경로 자체를 open하거나 `/proc/5082/fd/10`을 열어서는 안 된다.
현재 실행 환경 및 문서 작성 전/후 보호 대상 검증은 [documentation_verification.json](documentation_verification.json) 참조.
