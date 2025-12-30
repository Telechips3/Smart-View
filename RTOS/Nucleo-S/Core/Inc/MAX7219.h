#ifndef MAX7219_STM32_H
#define MAX7219_STM32_H

#include "main.h" // STM32 HAL 정의 포함

// --- 1. MAX7219 레지스터 정의 ---
#define MAX7219_REG_NOOP        0x00
#define MAX7219_REG_DIGIT0      0x01
#define MAX7219_REG_DIGIT1      0x02
#define MAX7219_REG_DIGIT2      0x03
#define MAX7219_REG_DIGIT3      0x04
#define MAX7219_REG_DIGIT4      0x05
#define MAX7219_REG_DIGIT5      0x06
#define MAX7219_REG_DIGIT6      0x07
#define MAX7219_REG_DIGIT7      0x08
#define MAX7219_REG_DECODEMODE  0x09
#define MAX7219_REG_INTENSITY   0x0A
#define MAX7219_REG_SCANLIMIT   0x0B
#define MAX7219_REG_SHUTDOWN    0x0C
#define MAX7219_REG_DISPLAYTEST 0x0F

// --- 2. 사용자 설정 ---
#define MAX7219_NUM_DEVICES     4     // 총 4개 모듈 (8x16 × 2개 헤드라이트)

// SPI 핸들러
extern SPI_HandleTypeDef hspi1;  // 왼쪽 헤드라이트용
extern SPI_HandleTypeDef hspi5;  // 오른쪽 헤드라이트용

// CS 핀 정의
#define MAX7219_CS1_PORT        MAX7219_CS1_GPIO_Port
#define MAX7219_CS1_PIN         MAX7219_CS1_Pin
#define MAX7219_CS2_PORT        MAX7219_CS2_GPIO_Port
#define MAX7219_CS2_PIN         MAX7219_CS2_Pin

// --- 3. 함수 프로토타입 ---
void MAX7219_Init(void);
void MAX7219_Write(uint8_t device_idx, uint8_t reg, uint8_t data);
void MAX7219_Clear(void);
void MAX7219_SetIntensity(uint8_t intensity); // 밝기 조절 (0~15)
void MAX7219_UpdateBuffer(uint8_t device_idx, uint8_t row, uint8_t data);
void MAX7219_Flush(void);

#endif
