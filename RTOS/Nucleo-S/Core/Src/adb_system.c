#include "adb_system.h"
#include "MAX7219.h"
// 현재 활성화된 컬럼 마스크 (1: 켜짐, 0: 꺼짐)
// 16비트: 0~15번 컬럼 제어
static uint16_t adb_mask = 0xFFFF;

void ADB_Init(void) {
    MAX7219_Init(); // SPI 및 MAX7219 설정
    ADB_SetX(-1); // 초기 상태: 그림자 없음 (모두 켜짐)
}

void ADB_SetX(int16_t center_x) {
    // 1. [마스크 초기화] 모두 켜짐(0xFF) 상태로 시작
    uint8_t mask_left_module = 0xFF;  // 왼쪽 모듈 (화면상 0~320)
    uint8_t mask_right_module = 0xFF; // 오른쪽 모듈 (화면상 320~640)

    // 2. 유효한 좌표 범위일 때만 계산
    if (center_x >= 0 && center_x <= CAM_WIDTH) {

        // (1) 중심 컬럼 인덱스 계산 (0 ~ 15)
        int center_col = (center_x * LED_COLS) / CAM_WIDTH;

        // 인덱스 범위 안전장치
        if (center_col < 0) center_col = 0;
        if (center_col >= LED_COLS) center_col = LED_COLS - 1;

        // (2) [가변 폭 설정]
        // 양쪽 끝(0번, 15번)이면 폭을 0(본인만)으로, 그 외 중간은 1(좌우 포함 3칸)로 설정
        int range = (center_col == 0 || center_col == (LED_COLS - 1)) ? 0 : 2;

        // (3) 주변부까지 포함하여 마스킹 (Loop)
        // range가 0이면 i는 0만 실행 (총 1개 꺼짐)
        // range가 1이면 i는 -1, 0, 1 실행 (총 3개 꺼짐)
        for (int i = -range; i <= range; i++) {
            int target_col = center_col + i;

            // 계산된 타겟이 전체 범위를 벗어나면 스킵 (안전장치)
            if (target_col < 0 || target_col >= LED_COLS) continue;

            // (4) 하드웨어 매핑 및 비트 끄기 (기존 로직 유지)

            // Case A: 왼쪽 화면 (0 ~ 7번 인덱스)
            if (target_col < 8) {
                // 하드웨어 역순 배치 보정: (7 - target_col)
                int bit_pos = 7 - target_col;
                mask_left_module &= ~(1 << bit_pos); // 해당 비트 0으로 설정(끔)
            }
            // Case B: 오른쪽 화면 (8 ~ 15번 인덱스)
            else {
                // 하드웨어 역순 배치 보정: (7 - (target_col - 8))
                int rel_col = target_col - 8;
                int bit_pos = 7 - rel_col;
                mask_right_module &= ~(1 << bit_pos); // 해당 비트 0으로 설정(끔)
            }
        }
    }

    // 5. [하드웨어 전송] Device 0, 1 할당
    // (기존 배선 유지: Device 1=오른쪽, Device 0=왼쪽 가정)
    uint8_t data_dev1 = mask_right_module;
    uint8_t data_dev0 = mask_left_module;

    // 6. 실제 SPI 버퍼 업데이트
    for (int row = 0; row < 8; row++) {
        uint8_t d0 = 0xFF;
        uint8_t d1 = 0xFF;

        // 상향등 영역(Row 0~6)만 마스킹 적용
        if (row < 6) {
            d0 &= data_dev0;
            d1 &= data_dev1;
        }

        MAX7219_UpdateBuffer(0, row, d0);
        MAX7219_UpdateBuffer(1, row, d1);
    }

    // 화면 갱신
    MAX7219_Flush();
}
