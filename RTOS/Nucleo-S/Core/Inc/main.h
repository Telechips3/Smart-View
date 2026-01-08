/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ROW3_Pin GPIO_PIN_2
#define ROW3_GPIO_Port GPIOE
#define COL2_Pin GPIO_PIN_3
#define COL2_GPIO_Port GPIOE
#define COL3_Pin GPIO_PIN_4
#define COL3_GPIO_Port GPIOE
#define COL7_Pin GPIO_PIN_5
#define COL7_GPIO_Port GPIOE
#define COL5_Pin GPIO_PIN_6
#define COL5_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define MAX7219_CS1_Pin GPIO_PIN_4
#define MAX7219_CS1_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define MAX7219_CS2_Pin GPIO_PIN_1
#define MAX7219_CS2_GPIO_Port GPIOG
#define ROW2_Pin GPIO_PIN_7
#define ROW2_GPIO_Port GPIOE
#define COL4_Pin GPIO_PIN_8
#define COL4_GPIO_Port GPIOE
#define ROW1_Pin GPIO_PIN_9
#define ROW1_GPIO_Port GPIOE
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define ROW6_Pin GPIO_PIN_8
#define ROW6_GPIO_Port GPIOC
#define ROW8_Pin GPIO_PIN_9
#define ROW8_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ROW7_Pin GPIO_PIN_10
#define ROW7_GPIO_Port GPIOC
#define ROW5_Pin GPIO_PIN_11
#define ROW5_GPIO_Port GPIOC
#define COL8_Pin GPIO_PIN_0
#define COL8_GPIO_Port GPIOD
#define COL6_Pin GPIO_PIN_1
#define COL6_GPIO_Port GPIOD
#define ROW4_Pin GPIO_PIN_2
#define ROW4_GPIO_Port GPIOD
#define COL1_Pin GPIO_PIN_3
#define COL1_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#pragma pack(push, 1)

extern SPI_HandleTypeDef hspi2;

typedef struct {
    uint8_t  header;    // 0xAA (Start Byte)
    uint8_t  detected;  // 0 : 앞, 1: 뒤
    uint8_t class_ID; // 무슨 객체인지
    float timestamp; // Time (2 Bytes)
    float    distance;  // Distance (4 Bytes)
    float    bbox_x;    // BBox X (4 Bytes)
    float    bbox_y;    // BBox Y (4 Bytes)
    float	bbox_h;
    float	bbox_w;
    uint8_t  checksum;  // CRC/XOR Checksum (1 Byte)
} SPI_Packet_t;

#pragma pack(pop)

// 2. 공용체 정의 (배열 <-> 구조체 변환용)
typedef union {
    uint8_t       buffer[sizeof(SPI_Packet_t)];
    SPI_Packet_t data;
} Shared_Buffer_t;

// 3. 패킷 사이즈 정의 (28 Bytes)
#define PACKET_SIZE sizeof(SPI_Packet_t)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
