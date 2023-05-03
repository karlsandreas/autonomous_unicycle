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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOC
#define LDERROR_Pin GPIO_PIN_15
#define LDERROR_GPIO_Port GPIOC
#define DEADMAN_GND_Pin GPIO_PIN_0
#define DEADMAN_GND_GPIO_Port GPIOC
#define DEADMAN_SW_Pin GPIO_PIN_1
#define DEADMAN_SW_GPIO_Port GPIOC
#define DEADMAN_LED_Pin GPIO_PIN_2
#define DEADMAN_LED_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */


#define GYRO_FS_CFG 0

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

#define ACC_FS_CFG 0

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


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
