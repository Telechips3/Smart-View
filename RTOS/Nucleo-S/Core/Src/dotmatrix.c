/* dotmatrix.c 최종본 */
#include "dotmatrix.h"

#include "main.h"

GPIO_TypeDef *col_port[] = { COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port, COL5_GPIO_Port, COL6_GPIO_Port, COL7_GPIO_Port, COL8_GPIO_Port };
uint16_t col_pin[] = { COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin, COL5_Pin, COL6_Pin, COL7_Pin, COL8_Pin };

GPIO_TypeDef *row_port[] = { ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port, ROW5_GPIO_Port, ROW6_GPIO_Port, ROW7_GPIO_Port, ROW8_GPIO_Port };
uint16_t row_pin[] = { ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin, ROW5_Pin, ROW6_Pin, ROW7_Pin, ROW8_Pin };

//이모티콘 표정
//static const uint8_t smile_face[8] = {
//    0b01111110, //   **** (머리)
//    0b11011011, //  * *
//    0b11011011, // * * * * (눈: 세로 3칸)
//    0b11011011, // * * * *
//    0b11111111, // * * * *
//    0b10111101, // * *
//    0b11000011, //  * ** * (입: 웃음)
//    0b01111110  //   **** (턱)
//};
//
//static const uint8_t normal_face[8] = {
//    0b01111110, //   ****
//    0b11011011, //  * *
//    0b11011011, // * * * * (눈)
//    0b11011011, // * * * *
//    0b11111111, // *******
//    0b11111111, // ******* (입꼬리 없음 -> 다 켜짐)
//    0b11000011, //  **** (입: 일자)
//    0b01111110  //   ****
//};
//
//// [3] 우는 얼굴 (울상)
//// 입꼬리가 내려감 (Row 5에 입 중앙, Row 6에 입꼬리)
//static const uint8_t sad_face[8] = {
//    0b01111110, //   ****
//    0b11011101, //  * *
//    0b11011011, // * * * * (눈)
//    0b11011011, // * * * *
//    0b11111111, //  **** (입 중앙: 위로 올라감)
//    0b11000011, // * * (입꼬리: 아래로 내려감)
//    0b10111101, // *******
//    0b01111110  //   ****
//};

static volatile uint8_t currentRow = 0; // 현재 스캔 중인 행
//static const uint8_t *currentImage;     // 현재 표시할 이미지 포인터
//static uint8_t imageIndex = 0;          // 이미지 순서 인덱스 (0:웃음, 1:울음, 2:평범)
//static uint16_t imageChangeCounter = 0; // 이미지 변경 카운터
static volatile uint8_t col_enable_mask = 0xFF;
// --- 3. 함수 구현 ---

void dotmatrix_init(void)
{
    // 1. 모든 LED 끄기 (초기 상태)
    for(int i=0; i<8; i++) {
        HAL_GPIO_WritePin(row_port[i], row_pin[i], GPIO_PIN_RESET); // Row 끔
        HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_SET);   // Col 끔
    }

    // 2. 마스크 초기화 (모두 켜짐 상태)
    col_enable_mask = 0xFF;
}

void dotmatrix_process_input(uint8_t rx_data)
{
    // 숫자 '1' ~ '8' 입력 시: 해당 열 끄기
    if (rx_data >= '1' && rx_data <= '8')
    {
        int col_idx = rx_data - '1'; // '1' -> 0번 인덱스
        col_enable_mask &= ~(1 << col_idx); // 해당 비트를 0으로 (OFF)
    }
    // 'r' 또는 'R' 입력 시: 초기화 (모두 켜기)
    else if (rx_data == 'r' || rx_data == 'R')
    {
        col_enable_mask = 0xFF; // 모든 비트 1
    }

}

// 1ms마다 호출될 스캔 함수
// main.c의 HAL_UART_RxCpltCallback에서 호출해주세요.
void dotmatrix_update(void)
{
    // 1. 이전 Row 끄기 (잔상 제거)
    uint8_t prevRow = (currentRow == 0) ? 7 : currentRow - 1;
    HAL_GPIO_WritePin(row_port[prevRow], row_pin[prevRow], GPIO_PIN_RESET);

    // 2. 데이터 준비 (항상 모든 LED ON = 0xFF)
    uint8_t data = 0xFF;

    // 3. Column 제어 (데이터 & 마스크 확인)
    for (int i = 0; i < 8; i++)
    {
        // (1) 기본적으로 켜져야 하고(data=1)
        // (2) UART로 끄라고 하지 않았어야 함(mask=1)
        if ( (data & (1 << i)) && (col_enable_mask & (1 << i)) )
        {
            HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_RESET); // 켜기 (Low)
        }
        else
        {
            HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_SET);   // 끄기 (High)
        }
    }

    // 4. 현재 Row 켜기
    HAL_GPIO_WritePin(row_port[currentRow], row_pin[currentRow], GPIO_PIN_SET);

    // 5. 다음 행으로 이동
    currentRow++;
    if (currentRow > 7) currentRow = 0;
}

#define CAM_WIDTH  640
// Matrix LED 열 개수 (현재 8열 사용)
#define LED_COLS   8

/**
  * @brief 바운딩 박스(X좌표, 너비)를 기반으로 LED 컬럼 제어 (ADB 기능)
  * @param bbox_x : 객체 중심 X 좌표 (0 ~ 640)
  * @param bbox_w : 객체 너비 (차량 크기)
  */
void dotmatrix_set_adb_pattern(int16_t bbox_x, uint16_t bbox_w)
{
    // 1. 기본 상태: 모든 LED 켜기 (High Beam 상태 = 11111111)
    uint8_t new_mask = 0xFF;

    // 2. 차량이 감지되었을 때만 계산 (너비가 0보다 클 때)
    if (bbox_w > 0)
    {
        // 2-1. 카메라 좌표계상 차량의 시작(왼쪽)과 끝(오른쪽) 위치 계산
        int x_min = bbox_x - (bbox_w / 2);
        int x_max = bbox_x + (bbox_w / 2);

        // 2-2. 좌표 -> LED 컬럼 인덱스로 변환 (Mapping)
        // 공식: (좌표 / 전체폭) * LED개수
        int col_start = (x_min * LED_COLS) / CAM_WIDTH;
        int col_end   = (x_max * LED_COLS) / CAM_WIDTH;

        // 2-3. 인덱스 범위 보호 (0 ~ 7 사이로 강제 고정)
        if (col_start < 0) col_start = 0;
        if (col_end >= LED_COLS) col_end = LED_COLS - 1;

        // 2-4. 해당 범위의 비트만 0(OFF)으로 설정 (Shadowing)
        for (int i = col_start; i <= col_end; i++)
        {
            new_mask &= ~(1 << i); // i번째 비트를 0으로 Clear
        }
    }

    // 3. 실제 제어 변수(static 변수) 업데이트
    // 이 변수는 dotmatrix_update() 함수에서 참조하여 LED를 켭니다.
    col_enable_mask = new_mask;
}
