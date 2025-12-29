#ifndef MAX7219_STM32_H
#define MAX7219_STM32_H

#include "main.h" // STM32 HAL 정의 포함

// --- 1. 업로드하신 파일의 레지스터 정의 유지 ---
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
#define MAX7219_NUM_DEVICES     2     // 8x16 매트릭스이므로 2개 연결
#define MAX7219_SPI_PORT        hspi1 // 사용하시는 SPI 핸들러
extern SPI_HandleTypeDef MAX7219_SPI_PORT;

// CS 핀 (CubeMX에서 설정한 핀 이름)
#define MAX7219_CS_PORT         MAX7219_CS_GPIO_Port
#define MAX7219_CS_PIN          MAX7219_CS_Pin

// --- 3. 함수 프로토타입 (C언어 스타일) ---
void MAX7219_Init(void);
void MAX7219_Write(uint8_t device_idx, uint8_t reg, uint8_t data);
void MAX7219_Clear(void);
void MAX7219_SetIntensity(uint8_t intensity); // 밝기 조절 (0~15)
void MAX7219_UpdateBuffer(uint8_t device_idx, uint8_t row, uint8_t data); // 버퍼만 업데이트
void MAX7219_Flush(void); // 버퍼 내용을 화면에 쏘기

#endif
