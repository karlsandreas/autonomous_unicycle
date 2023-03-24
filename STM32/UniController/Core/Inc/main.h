/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define GYRO_FS_CFG 3

// Gyroscope scale

// GYRO_FS is in rad/s
#if GYRO_FS_CFG == 0
	#define GYRO_FS_DEG 250
#elif GYRO_FS_CFG == 1
	#define GYRO_FS_DEG 500
#elif GYRO_FS_CFG == 2
	#define GYRO_FS_DEG 1000
#elif GYRO_FS_CFG == 3
#define GYRO_FS_DEG 2000
#endif
#define GYRO_FS ((float)GYRO_FS_DEG / 180 * 3.1415)

// Accelerometer scale

#define ACC_FS_CFG 3

#if ACC_FS_CFG == 0
	#define ACC_FS_G 2
#elif ACC_FS_CFG == 1
	#define ACC_FS_G 4
#elif ACC_FS_CFG == 2
	#define ACC_FS_G 8
#elif ACC_FS_CFG == 3
#define ACC_FS_G 16
#endif
// ACC_FS in m/s²
#define ACC_FS (ACC_FS_G*9.82)

// Digital low-pass filter

#define DLPF_CFG 6

#if DLPF_CFG == 0
	#define ACC_BW 260
	#define GYRO_BW 256
#elif DLPF_CFG == 1
	#define ACC_BW 184
	#define GYRO_BW 188
#elif DLPF_CFG == 2
	#define ACC_BW 94
	#define GYRO_BW 98
#elif DLPF_CFG == 3
	#define ACC_BW 44
	#define GYRO_BW 42
#elif DLPF_CFG == 4
	#define ACC_BW 21
	#define GYRO_BW 20
#elif DLPF_CFG == 5
	#define ACC_BW 10
	#define GYRO_BW 10
#elif DLPF_CFG == 6
	#define ACC_BW 5
	#define GYRO_BW 5
#endif

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define BTN1_T_Pin GPIO_PIN_15
#define BTN1_T_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define BTN1_O_Pin GPIO_PIN_6
#define BTN1_O_GPIO_Port GPIOC
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
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define BTN1_I_Pin GPIO_PIN_8
#define BTN1_I_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

// Red LED LD3 showing error
#define LDERROR_Pin LD3_Pin
#define LDERROR_GPIO_Port LD3_GPIO_Port



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
