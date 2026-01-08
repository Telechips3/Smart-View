#include "rear_display.h"

GPIO_TypeDef *col_port[] = { COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port, COL5_GPIO_Port, COL6_GPIO_Port, COL7_GPIO_Port, COL8_GPIO_Port };
uint16_t col_pin[] = { COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin, COL5_Pin, COL6_Pin, COL7_Pin, COL8_Pin };

GPIO_TypeDef *row_port[] = { ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port, ROW5_GPIO_Port, ROW6_GPIO_Port, ROW7_GPIO_Port, ROW8_GPIO_Port };
uint16_t row_pin[] = { ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin, ROW5_Pin, ROW6_Pin, ROW7_Pin, ROW8_Pin };

// 웃는 표정
static const uint8_t smile_face[8] = {
    0x00, // 00000000
    0x66, // 01100110  (큰 눈)
    0x66, // 01100110
    0x00, // 00000000
    0x81, // 10000001  (입꼬리)
    0x42, // 01000010
    0x3C, // 00111100  (입 중앙)
    0x00  // 00000000
};
// 무표정
static const uint8_t normal_face[8] = {
    0x00, // 00000000
    0x66, // 01100110  (눈)
    0x66, // 01100110
    0x00, // 00000000
    0x00, // 00000000
    0x7E, // 01111110  (긴 일자 입)
    0x00, // 00000000
    0x00  // 00000000
};

// 우는 얼굴 (울상)
static const uint8_t sad_face[8] = {
    0x00, // 00000000
    0x66, // 01100110  (눈)
    0x66, // 01100110
    0x00, // 00000000  (눈물 주르륵)
    0x00, // 00100100  (눈물 주르륵)
    0x00, // 00000000
    0x3C, // 00111100  (입 - 찡그림)
    0x42  // 01000010  (입꼬리 처짐)
};

static volatile uint8_t currentRow = 0;       // 현재 스캔 중인 행
static const uint8_t *current_face_ptr = smile_face; // 현재 출력할 이미지

void RearDisplay_Init(void)
{
    // 모든 LED 끄기
    for(int i=0; i<8; i++) {
        HAL_GPIO_WritePin(row_port[i], row_pin[i], GPIO_PIN_RESET); // Row Low (OFF)
        HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_SET);   // Col High (OFF)
    }
    current_face_ptr = smile_face;
}

// 거리값에 따라 표정 변경 로직
void RearDisplay_SetDistance(float distance)
{
    if (distance < 50.0f) {
        current_face_ptr = sad_face;    // 위험 (너무 가까움)
    } else if (distance < 80.0f) {
        current_face_ptr = normal_face; // 주의
    } else {
        current_face_ptr = smile_face;  // 안전
    }
}

// 스캔 함수
void RearDisplay_UpdateScan(void)
{
    // (1) 이전 잔상 제거 (Row OFF, Col OFF)
    HAL_GPIO_WritePin(row_port[currentRow], row_pin[currentRow], GPIO_PIN_RESET);
    for(int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_SET);
    }

    // (2) 다음 행 준비
    currentRow++;
    if (currentRow > 7) currentRow = 0;

    // (3) 현재 그릴 데이터 가져오기
    uint8_t data_row = current_face_ptr[currentRow];

    // (4) Column 데이터 세팅 (비트 확인)
    for (int i = 0; i < 8; i++)
    {
        // 비트가 1이면 켠다 (Low), 0이면 끈다 (High)
        if (data_row & (1 << i)) {
            HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(col_port[i], col_pin[i], GPIO_PIN_SET);
        }
    }

    // (5) Row 켜기 (스캔 시작)
    HAL_GPIO_WritePin(row_port[currentRow], row_pin[currentRow], GPIO_PIN_SET);
}
