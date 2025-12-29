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
    // 1. [좌우 모듈 마스크 초기화] 기본적으로 모두 켬(0xFF)
    uint8_t mask_left_module = 0xFF;  // 왼쪽 모듈 (화면상 0~320)
    uint8_t mask_right_module = 0xFF; // 오른쪽 모듈 (화면상 320~640)

    // 2. 유효한 좌표 범위일 때만 계산
    if (center_x >= 0 && center_x <= CAM_WIDTH) {

        // 3. [좌표 변환] 0~640 -> 0~15 (선형 변환)
        int target_col = (center_x * LED_COLS) / CAM_WIDTH;
        if (target_col < 0) target_col = 0;
        if (target_col >= LED_COLS) target_col = LED_COLS - 1;

        // 4. [매핑 보정] 하드웨어 방향에 맞춰 비트 위치 뒤집기

        // Case A: 왼쪽 화면 (0 ~ 7번 인덱스)
        if (target_col < 8) {
            // 사용자의 현상: 0을 넣으면 가운데(오른쪽 끝)가 꺼짐 -> 역순 배치됨
            // 해결: (7 - target_col)로 뒤집어서 0일 때 왼쪽 끝이 꺼지게 함
            int bit_pos = 7 - target_col;
            mask_left_module &= ~(1 << bit_pos);
        }
        // Case B: 오른쪽 화면 (8 ~ 15번 인덱스)
        else {
            // 사용자의 현상: 320(8번)을 넣으면 오른쪽 끝이 꺼짐 -> 역순 배치됨
            // 해결: (7 - (target_col - 8))로 뒤집어서 8일 때 왼쪽(가운데)이 꺼지게 함
            int rel_col = target_col - 8;
            int bit_pos = 7 - rel_col; // 여기서도 뒤집음
            mask_right_module &= ~(1 << bit_pos);
        }
    }

    // 5. [하드웨어 전송] Device 0, 1 할당
    // 배선에 따라 Device 0이 왼쪽인지 오른쪽인지 결정됩니다.
    // 만약 좌우가 통째로 바뀌어 있다면 아래 두 줄의 변수(mask_...)를 서로 맞바꾸세요.

    uint8_t data_dev1 = mask_right_module; // 가정: Device 0이 오른쪽 모듈
    uint8_t data_dev0 = mask_left_module;  // 가정: Device 1이 왼쪽 모듈

    // 6. 실제 SPI 버퍼 업데이트
    for (int row = 0; row < 8; row++) {
        uint8_t d0 = 0xFF;
        uint8_t d1 = 0xFF;

        // 상향등 영역(Row 0~3)만 마스킹 적용
        if (row < 4) {
            d0 &= data_dev0;
            d1 &= data_dev1;
        }

        MAX7219_UpdateBuffer(0, row, d0);
        MAX7219_UpdateBuffer(1, row, d1);
    }

    MAX7219_Flush();
}
