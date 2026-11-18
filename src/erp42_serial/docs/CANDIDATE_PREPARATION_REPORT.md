# 독립 후보 빌드·복구 준비 완료

현재 미커밋 소스를 영구 스냅샷으로 보존해 원본 외부에서 후보를 빌드했다.
오프라인 85/85, ASan/UBSan 85/85, 격리 ROS 48/48 및 기존 exe의 복구 Mock 실행이 통과했다.
기존 Serial PID5082, 원본 build/devel, 전체 백업, 기존 소스, Git index·브랜치·Remote를 유지했다. 실차 제어는 실행하지 않았다.
실차 Manual↔Auto는 미확인 하드웨어 제어 계약 때문에 BLOCKED다.

- [최종 보고서](/home/autonav/ERP42_preparation_20261010__ltm711z/FINAL_REPORT.md)
- [사용자 ROS 가이드](/home/autonav/ERP42_preparation_20261010__ltm711z/docs/USER_ROS_GUIDE.md)
- [제어 계약 근거](/home/autonav/ERP42_preparation_20261010__ltm711z/docs/HARDWARE_CONTRACT.md)
- [복구 절차](/home/autonav/ERP42_preparation_20261010__ltm711z/rollback/RESTORE.md)
- [최종 환경 보호 검증](/home/autonav/ERP42_preparation_20261010__ltm711z/environment_verification.json)

다음 단계는 기존 Manual 정지 상태를 유지한 수동 제동/E-Stop feedback·command 및 사건 시각의 읽기 전용 동기 기록이다. 이 기록만으로 Auto Brake/E-Stop 명령 효과를 확정하지 않는다.
