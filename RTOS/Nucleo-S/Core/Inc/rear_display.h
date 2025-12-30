#ifndef INC_REAR_DISPLAY_H_
#define INC_REAR_DISPLAY_H_

#include "main.h"

// 이모티콘 상태 정의
typedef enum {
    FACE_SMILE = 0,  // 안전 (웃음)
    FACE_NORMAL,     // 주의 (무표정)
    FACE_SAD         // 위험 (울상)
} FaceState;

void RearDisplay_Init(void);
void RearDisplay_UpdateScan(void); // 1ms마다 호출되어야 함 (스캔용)
void RearDisplay_SetDistance(float distance); // 거리값에 따라 표정 변경

#endif /* INC_REAR_DISPLAY_H_ */
