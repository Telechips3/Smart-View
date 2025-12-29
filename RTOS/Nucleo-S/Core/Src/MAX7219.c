#include "MAX7219.h"

// 디스플레이 버퍼 (화면 상태 저장용)
static uint8_t DisplayBuffer[MAX7219_NUM_DEVICES][8];

// 내부 함수: CS 핀 제어
static void CS_Low(void) {
    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET);
}

static void CS_High(void) {
    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);
}

// 핵심: 캐스케이드(Daisy-Chain) 지원 데이터 전송
void MAX7219_Write(uint8_t device_idx, uint8_t reg, uint8_t data) {
    CS_Low();

    // Daisy Chain: 먼 쪽(인덱스 큰 쪽) 데이터부터 밀어넣기
    for (int i = MAX7219_NUM_DEVICES - 1; i >= 0; i--) {
        if (i == device_idx) {
            HAL_SPI_Transmit(&MAX7219_SPI_PORT, &reg, 1, 10);
            HAL_SPI_Transmit(&MAX7219_SPI_PORT, &data, 1, 10);
        } else {
            // 다른 칩에는 No-Op(0x00) 전송하여 상태 유지
            uint8_t noop_reg = MAX7219_REG_NOOP;
            uint8_t noop_data = 0x00;
            HAL_SPI_Transmit(&MAX7219_SPI_PORT, &noop_reg, 1, 10);
            HAL_SPI_Transmit(&MAX7219_SPI_PORT, &noop_data, 1, 10);
        }
    }

    CS_High();
}

void MAX7219_Init(void) {
    for (int i = 0; i < MAX7219_NUM_DEVICES; i++) {
        MAX7219_Write(i, MAX7219_REG_SHUTDOWN, 0x01);    // 켜기
        MAX7219_Write(i, MAX7219_REG_DECODEMODE, 0x00);  // 패턴 모드
        MAX7219_Write(i, MAX7219_REG_SCANLIMIT, 0x07);   // 8줄 스캔
        MAX7219_Write(i, MAX7219_REG_INTENSITY, 0x05);   // 밝기 중간
        MAX7219_Write(i, MAX7219_REG_DISPLAYTEST, 0x00); // 테스트 끔
    }
    MAX7219_Clear();
}

void MAX7219_Clear(void) {
    for (int dev = 0; dev < MAX7219_NUM_DEVICES; dev++) {
        for (int row = 0; row < 8; row++) {
            DisplayBuffer[dev][row] = 0x00;
            MAX7219_Write(dev, MAX7219_REG_DIGIT0 + row, 0x00);
        }
    }
}

void MAX7219_SetIntensity(uint8_t intensity) {
    if (intensity > 0x0F) intensity = 0x0F;
    for (int i = 0; i < MAX7219_NUM_DEVICES; i++) {
        MAX7219_Write(i, MAX7219_REG_INTENSITY, intensity);
    }
}

// 버퍼에 데이터 기록 (화면엔 아직 반영 안 됨)
void MAX7219_UpdateBuffer(uint8_t device_idx, uint8_t row, uint8_t data) {
    if (device_idx >= MAX7219_NUM_DEVICES || row >= 8) return;
    DisplayBuffer[device_idx][row] = data;
}

// 버퍼 내용을 한 번에 화면으로 전송 (효율적)
void MAX7219_Flush(void) {
    for (int row = 0; row < 8; row++) {
        CS_Low();
        // 모든 칩의 같은 Row를 동시에 처리
        for (int dev = MAX7219_NUM_DEVICES - 1; dev >= 0; dev--) {
             uint8_t reg = MAX7219_REG_DIGIT0 + row;
             uint8_t data = DisplayBuffer[dev][row];
             HAL_SPI_Transmit(&MAX7219_SPI_PORT, &reg, 1, 10);
             HAL_SPI_Transmit(&MAX7219_SPI_PORT, &data, 1, 10);
        }
        CS_High();
    }
}
