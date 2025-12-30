#include "adb_system.h"
#include "MAX7219.h"

/**
 * @brief ADB 시스템 초기화
 */
void ADB_Init(void) {
    MAX7219_Init(); // MAX7219 초기화
    ADB_SetX(-1);   // 초기 상태: 모든 LED 켜짐
}

/**
 * @brief 차량 X좌표에 따라 헤드라이트 제어
 * @param center_x: 차량 중심 X좌표 (0~640)
 *                  -1: 모든 LED 켜기 (차량 없음)
 *                  0~319: 왼쪽 헤드라이트만 제어
 *                  320~640: 오른쪽 헤드라이트만 제어
 *
 * 하드웨어 구조:
 * ┌─────────────────────┬─────────────────────┐
 * │  왼쪽 헤드라이트     │  오른쪽 헤드라이트   │
 * │     (8x16)          │     (8x16)          │
 * ├──────────┬──────────┼──────────┬──────────┤
 * │ 모듈 0   │ 모듈 1   │ 모듈 2   │ 모듈 3   │
 * │ (0~7열)  │(8~15열)  │ (0~7열)  │(8~15열)  │
 * └──────────┴──────────┴──────────┴──────────┘
 */
void ADB_SetX(int16_t center_x) {
    // 1. 초기화: 모든 마스크를 켜진 상태(0xFF)로 시작
    uint8_t mask_left[2] = {0xFF, 0xFF};   // 왼쪽 헤드라이트 2개 모듈
    uint8_t mask_right[2] = {0xFF, 0xFF};  // 오른쪽 헤드라이트 2개 모듈

    // 2. 차량 감지 시 해당 위치 LED 끄기
    if (center_x >= 0 && center_x <= CAM_WIDTH) {

        // ==========================================================
        // CASE 1: 왼쪽 화면(0 ~ 319)에 차량 있음 → 왼쪽 헤드라이트만 제어
        // ==========================================================
        if (center_x < HALF_WIDTH) {
            // 0~319 좌표 → 0~15 열 인덱스로 매핑
            int col_idx = (center_x * 16) / HALF_WIDTH;

            // 인덱스 범위 보호
            if (col_idx < 0) col_idx = 0;
            if (col_idx >= 16) col_idx = 15;

            // 그림자 범위: ±1칸 (총 3칸 끔)
            int range = 1;

            for (int i = -range; i <= range; i++) {
                int target = col_idx + i;
                if (target >= 0 && target < 16) {
                    // 16열을 2개 모듈로 분할
                    if (target < 8) {
                        // 왼쪽 헤드라이트 모듈 0 (0~7열)
                        mask_left[0] &= ~(1 << (7 - target));
                    } else {
                        // 왼쪽 헤드라이트 모듈 1 (8~15열)
                        int col = target - 8;
                        mask_left[1] &= ~(1 << (7 - col));
                    }
                }
            }
        }
        // ==========================================================
        // CASE 2: 오른쪽 화면(320 ~ 640)에 차량 있음 → 오른쪽 헤드라이트만 제어
        // ==========================================================
        else {
            // 320~640 좌표 → 0~15 열 인덱스로 매핑
            int col_idx = ((center_x - HALF_WIDTH) * 16) / HALF_WIDTH;

            if (col_idx < 0) col_idx = 0;
            if (col_idx >= 16) col_idx = 15;

            int range = 1;

            for (int i = -range; i <= range; i++) {
                int target = col_idx + i;
                if (target >= 0 && target < 16) {
                    if (target < 8) {
                        // 오른쪽 헤드라이트 모듈 2 (0~7열)
                        mask_right[0] &= ~(1 << (7 - target));
                    } else {
                        // 오른쪽 헤드라이트 모듈 3 (8~15열)
                        int col = target - 8;
                        mask_right[1] &= ~(1 << (7 - col));
                    }
                }
            }
        }
    }

    // 3. 버퍼 업데이트
    for (int row = 0; row < 8; row++) {
        uint8_t d_left0 = 0xFF;
        uint8_t d_left1 = 0xFF;
        uint8_t d_right0 = 0xFF;
        uint8_t d_right1 = 0xFF;

        // 상향등 영역(Row 0~5)만 마스킹 적용
        // Row 6~7은 하향등이므로 항상 켜짐
        if (row < 6) {
            d_left0 &= mask_left[0];
            d_left1 &= mask_left[1];
            d_right0 &= mask_right[0];
            d_right1 &= mask_right[1];
        }

        // 왼쪽 헤드라이트 (모듈 0, 1)
        MAX7219_UpdateBuffer(0, row, d_left0);
        MAX7219_UpdateBuffer(1, row, d_left1);

        // 오른쪽 헤드라이트 (모듈 2, 3)
        MAX7219_UpdateBuffer(2, row, d_right0);
        MAX7219_UpdateBuffer(3, row, d_right1);
    }

    // 4. 화면에 반영
    MAX7219_Flush();
}
