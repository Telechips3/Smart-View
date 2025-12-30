#include "MAX7219.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi5;

// [디스플레이 버퍼 구조]
// 0: 왼쪽 헤드라이트의 앞부분 (Col 0~7)  -> 칩1 (Near)
// 1: 왼쪽 헤드라이트의 뒷부분 (Col 8~15) -> 칩2 (Far)
// 2: 오른쪽 헤드라이트의 앞부분 (Col 0~7) -> 칩3 (Near)
// 3: 오른쪽 헤드라이트의 뒷부분 (Col 8~15) -> 칩4 (Far)
static uint8_t DisplayBuffer[4][8];

// CS 핀 제어 매크로
#define CS1_LOW()   HAL_GPIO_WritePin(MAX7219_CS1_GPIO_Port, MAX7219_CS1_Pin, GPIO_PIN_RESET)
#define CS1_HIGH()  HAL_GPIO_WritePin(MAX7219_CS1_GPIO_Port, MAX7219_CS1_Pin, GPIO_PIN_SET)

#define CS2_LOW()   HAL_GPIO_WritePin(MAX7219_CS2_GPIO_Port, MAX7219_CS2_Pin, GPIO_PIN_RESET)
#define CS2_HIGH()  HAL_GPIO_WritePin(MAX7219_CS2_GPIO_Port, MAX7219_CS2_Pin, GPIO_PIN_SET)

// 초기화 함수 (모든 모듈을 동일하게 설정)
// Daisy-Chain 구조이므로, 명령어를 2번 연속 보내서 두 칩 모두 설정되게 함
void MAX7219_Init(void) {
    uint8_t init_cmds[][2] = {
        {MAX7219_REG_SHUTDOWN, 0x01},   // 켜기
        {MAX7219_REG_DECODEMODE, 0x00}, // 패턴 모드
        {MAX7219_REG_SCANLIMIT, 0x07},  // 8줄 전면 사용
        {MAX7219_REG_INTENSITY, 0x05},  // 밝기 중간
        {MAX7219_REG_DISPLAYTEST, 0x00} // 테스트 모드 끔
    };

    // 설정 명령어 개수만큼 반복
    for (int i = 0; i < 5; i++) {
        uint8_t reg = init_cmds[i][0];
        uint8_t data = init_cmds[i][1];

        // 1. 왼쪽 헤드라이트 (SPI1) 초기화
        CS1_LOW();
        // 칩2(Far)에게 전송
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi1, &data, 1, 10);
        // 칩1(Near)에게 전송 (칩2로 밀려감)
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi1, &data, 1, 10);
        CS1_HIGH();

        // 2. 오른쪽 헤드라이트 (SPI5) 초기화
        CS2_LOW();
        // 칩4(Far)에게 전송
        HAL_SPI_Transmit(&hspi5, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi5, &data, 1, 10);
        // 칩3(Near)에게 전송
        HAL_SPI_Transmit(&hspi5, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi5, &data, 1, 10);
        CS2_HIGH();
    }

    MAX7219_Clear();
}

// 화면 지우기
void MAX7219_Clear(void) {
    for(int i=0; i<4; i++) {
        for(int row=0; row<8; row++) {
            DisplayBuffer[i][row] = 0x00;
        }
    }
    MAX7219_Flush();
}

// 버퍼 업데이트 (메모리에만 쓰기)
void MAX7219_UpdateBuffer(uint8_t device_idx, uint8_t row, uint8_t data) {
    if (device_idx >= 4 || row >= 8) return;
    DisplayBuffer[device_idx][row] = data;
}

// ★ 핵심: 데이지 체인 전송 (Daisy-Chain Flush)
// 데이터가 칩을 통과해서 뒤로 밀려가는 원리를 이용
// 순서: [먼 쪽 데이터(Far)] -> [가까운 쪽 데이터(Near)]
void MAX7219_Flush(void) {
    for (int row = 0; row < 8; row++) {
        uint8_t reg = MAX7219_REG_DIGIT0 + row;

        // ====================================================
        // 1. 왼쪽 헤드라이트 (SPI1) 한 방에 쏘기
        // 구조: [MCU] -> [모듈0(Near)] -> [모듈1(Far)]
        // ====================================================
        CS1_LOW();

        // (1) 먼저 '모듈1(Far, Col 8~15)' 데이터를 보냄 -> 모듈0을 거쳐 모듈1로 밀려감
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi1, &DisplayBuffer[1][row], 1, 10);

        // (2) 그 다음 '모듈0(Near, Col 0~7)' 데이터를 보냄 -> 모듈0에 남음
        HAL_SPI_Transmit(&hspi1, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi1, &DisplayBuffer[0][row], 1, 10);

        CS1_HIGH(); // 래치! (두 모듈 동시 갱신)


        // ====================================================
        // 2. 오른쪽 헤드라이트 (SPI5) 한 방에 쏘기
        // 구조: [MCU] -> [모듈2(Near)] -> [모듈3(Far)]
        // ====================================================
        CS2_LOW();

        // (1) '모듈3(Far, Col 8~15)' 데이터 전송
        HAL_SPI_Transmit(&hspi5, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi5, &DisplayBuffer[3][row], 1, 10);

        // (2) '모듈2(Near, Col 0~7)' 데이터 전송
        HAL_SPI_Transmit(&hspi5, &reg, 1, 10);
        HAL_SPI_Transmit(&hspi5, &DisplayBuffer[2][row], 1, 10);

        CS2_HIGH(); // 래치!
    }
}
