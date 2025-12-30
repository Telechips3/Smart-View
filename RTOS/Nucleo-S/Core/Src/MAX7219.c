#include "MAX7219.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi5;
// 디스플레이 버퍼 (화면 상태 저장용)
static uint8_t DisplayBuffer[2][8];

// 내부 함수: CS 핀 제어
#define CS1_LOW()   HAL_GPIO_WritePin(MAX7219_CS1_GPIO_Port, MAX7219_CS1_Pin, GPIO_PIN_RESET)
#define CS1_HIGH()  HAL_GPIO_WritePin(MAX7219_CS1_GPIO_Port, MAX7219_CS1_Pin, GPIO_PIN_SET)

// 2. 오른쪽 (새로 추가한 PG1)
#define CS2_LOW()   HAL_GPIO_WritePin(MAX7219_CS2_GPIO_Port, MAX7219_CS2_Pin, GPIO_PIN_RESET) // PG1
#define CS2_HIGH()  HAL_GPIO_WritePin(MAX7219_CS2_GPIO_Port, MAX7219_CS2_Pin, GPIO_PIN_SET)   // PG1


// 핵심: 캐스케이드(Daisy-Chain) 지원 데이터 전송
void MAX7219_Write(uint8_t device_idx, uint8_t reg, uint8_t data) {
    if(device_idx == 0)
    {
    	//왼쪽 헤드라이트 사용
    	CS1_LOW();
    	HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
    	HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    	CS1_HIGH();
    }
    else if(device_idx == 1)
    {
    	CS2_LOW();
    	HAL_SPI_Transmit(&hspi5, &reg, 1, 10);
    	HAL_SPI_Transmit(&hspi5, &data, 1, 10);
    	CS2_HIGH();
    }
}

void MAX7219_Init(void) {
    // 초기화 명령어 세트
    uint8_t init_cmds[][2] = {
        {MAX7219_REG_SHUTDOWN, 0x01},   // 켜기
        {MAX7219_REG_DECODEMODE, 0x00}, // 패턴 모드
        {MAX7219_REG_SCANLIMIT, 0x07},  // 8줄 사용
        {MAX7219_REG_INTENSITY, 0x05},  // 밝기 (0x00 ~ 0x0F 조절 가능)
        {MAX7219_REG_DISPLAYTEST, 0x00} // 테스트 끔
    };

    // 왼쪽(0), 오른쪽(1) 모듈 모두 설정 전송
    for (int dev = 0; dev < 2; dev++) {
        for (int cmd = 0; cmd < 5; cmd++) {
            MAX7219_Write(dev, init_cmds[cmd][0], init_cmds[cmd][1]);
        }
    }

    MAX7219_Clear();
}

void MAX7219_Clear(void) {
    for(int row=0; row<8; row++) {
        DisplayBuffer[0][row] = 0x00;
        DisplayBuffer[1][row] = 0x00;
    }
    MAX7219_Flush();
}

void MAX7219_SetIntensity(uint8_t intensity) {
    if (intensity > 0x0F) intensity = 0x0F;
    MAX7219_Write(0, MAX7219_REG_INTENSITY, intensity);
    MAX7219_Write(1, MAX7219_REG_INTENSITY, intensity);
}

// 버퍼에 데이터 기록 (화면엔 아직 반영 안 됨)
void MAX7219_UpdateBuffer(uint8_t device_idx, uint8_t row, uint8_t data) {
    if (device_idx >= 2 || row >= 8) return;
    DisplayBuffer[device_idx][row] = data;
}

// 버퍼 내용을 한 번에 화면으로 전송 (효율적)
void MAX7219_Flush(void) {
    for (int row = 0; row < 8; row++) {
        uint8_t reg = MAX7219_REG_DIGIT0 + row;

        //왼쪽 헤드라이트
        uint8_t data_left = DisplayBuffer[0][row];
        CS1_LOW();
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi1, &data_left, 1, 10);
        CS1_HIGH();

        //오른쪽 헤드라이트
        uint8_t data_right = DisplayBuffer[1][row];
        CS2_LOW(); // PG1 Low
        HAL_SPI_Transmit(&hspi5, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi5, &data_right, 1, 10);
        CS2_HIGH(); // PG1 High
    }
}
