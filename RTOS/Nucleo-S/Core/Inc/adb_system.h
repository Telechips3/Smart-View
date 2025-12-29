#ifndef ADB_SYSTEM_H
#define ADB_SYSTEM_H

#include "main.h"
#include "MAX7219.h" // MAX7219 드라이버 포함

// 카메라 및 LED 설정
#define CAM_WIDTH   640
#define LED_COLS    16      // 8x8 모듈 2개 = 16열

// 함수 프로토타입
void ADB_Init(void);
void ADB_SetX(int16_t center_x);
#endif
