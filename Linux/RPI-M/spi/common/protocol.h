#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

typedef struct {
    uint8_t  header;      // 0xAA (시작 바이트)
    uint8_t  detected;    // 탐지 여부
    uint8_t  class_ID;    // 객체 종류
    float    timestamp;   // 시간 (4 Bytes)
    float    distance;    // 거리 (4 Bytes)
    float    bbox_x;      // 좌표 X (4 Bytes)
    float    bbox_y;      // 좌표 Y (4 Bytes)
    float    bbox_h;      // 높이 (4 Bytes)
    float    bbox_w;      // 너비 (4 Bytes)
    uint8_t  checksum;    // XOR 체크섬
} __attribute__((packed)) UART_Packet_t;

#define PACKET_SIZE sizeof(UART_Packet_t) // 28 Bytes
#define DEVICE_NAME "stm32_spi"

#endif
