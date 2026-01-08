#ifndef ADB_SYSTEM_H
#define ADB_SYSTEM_H

#include "main.h"
#include "MAX7219.h" // MAX7219 드라이버 포함

// 카메라 및 LED 설정
#define CAM_WIDTH 640
#define HALF_WIDTH 320
#define MODULE_COLS 8  // 각 모듈당 8열

// 함수 프로토타입
void ADB_Init(void);
void ADB_SetX(int16_t center_x, int8_t range);
#endif
