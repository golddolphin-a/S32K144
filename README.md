# S32K144

개발 단계
Phase 1: SRAM 테스트 버전
목적: 빠른 반복 테스트, 디버깅
특징:

Linker script: CODE/DATA 모두 SRAM 배치
CVD로 직접 download & run
Flash erase 불필요 → 빠른 iteration

검증 항목:

CAN 초기화 성공
타이머 인터럽트 주기 확인
TX/RX 정상 동작
var.draw 파형 확인

Phase 2: Flash 배포 버전
목적: 실제 demo용 standalone 동작
특징:

Linker script: CODE는 Flash, DATA는 SRAM
Startup code 추가 (Flash → SRAM copy)
Reset 후 자동 실행
