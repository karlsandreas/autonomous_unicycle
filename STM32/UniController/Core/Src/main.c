/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>

#include "vesc_com.h"
#include "queue.h"
#include "ctrl/common.h"
#include "ctrl/regulator.h"
#include "ctrl/kalman_filter.h"
#include "buf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x300400c0
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x300400c0))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static Queue MAIN_QUEUE;

uint16_t MPU_ADDR = 0x68;

uint32_t ms_counter;

// (will take about 1000 hours to overflow)
uint32_t khz_counter = 0;

#define ACC_FREQ 100
#define VESC_FREQ 50
#define DUMP_FREQ 100
#define STEP_FREQ 100

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) { // htim3 ticks once every us, elapses once every ms
		ms_counter++;
	}
	if (htim == &htim4 ) {
		// TODO: Put these on different timers?

		khz_counter++;

		if (khz_counter % (1000 / ACC_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_REQ_ACC });
		}
		if (khz_counter % (1000 / VESC_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_REQ_VESC });
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_FLUSH_VESC });
		}
		if (khz_counter % (1000 / DUMP_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_SEND_DEBUG });
		}
		if (khz_counter % (1000 / STEP_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_TIME_STEP });
		}

	}
	if (htim == &htim5) {
		// queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_SEND_DEBUG });
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		vesc_uart_cb_txcplt(huart);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		vesc_uart_cb_rxcplt(huart);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {

		vesc_uart_cb_rxcplt(huart);
	}
}

// This will overflow after 2^32/10^6s ≈ 4300s ≈ 1h11m
// Overflow should be handled by the get_dt and reset_dt
uint32_t us_since_startup() {
	uint32_t us_counter = ms_counter * 1000;
	uint32_t us_timer = htim3.Instance->CNT;
	return us_counter + us_timer;
}

uint32_t last_step = 0;
uint32_t get_and_reset_dt_us() {
	uint32_t now = us_since_startup();
	uint32_t dt = now - last_step; // If now < last_step, then we have overflowed. This should still get the right value
	last_step = now;
	return dt;
}

// Checks if the dead man's switch is both connected and pressed
bool dead_mans_switch_activated() {
	if (HAL_GPIO_ReadPin(BTN1_T_GPIO_Port, BTN1_T_Pin) == GPIO_PIN_RESET) {
		return false;
	}
	if (HAL_GPIO_ReadPin(BTN1_I_GPIO_Port, BTN1_I_Pin) == GPIO_PIN_SET) {
		return false;
	}
	return true;
}

// Puts the MPU6050 into active mode, with the appropriate CFGs from main.h set
void setup_mpu(I2C_HandleTypeDef *i2c, uint16_t acc_addr) {
	uint8_t pwr_mgnt_data[1] = {0x00}; // sleep = 0, cycle = 0
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x6b, 1, pwr_mgnt_data, 1, 200000); // Disable sleep

	uint8_t config[1] = {(0x0 << 3) | DLPF_CFG}; // fsync = 0, dlpf = 6 (5Hz)
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x1a, 1, config, 1, 200000);

	uint8_t gyro_config[1] = {(0x0 << 5) | (GYRO_FS_CFG << 3)}; // no self-test
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x1b, 1, gyro_config, 1, 200000);

	uint8_t acc_config[1] = {(0x0 << 5) | (ACC_FS_CFG << 3)}; // no self-test
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x1c, 1, acc_config, 1, 200000);
}

// TODO: Do this with interrupt
AccData get_accelerometer_data(I2C_HandleTypeDef *i2c, uint16_t acc_addr) {
	// 0x43 = gyro_xout_h
	// 0x44 = gyro_xout_
	uint8_t gyro_data[6] = {0x69, 0x69, 0x69, 0x69, 0x69, 0x69};
	HAL_I2C_Mem_Read(&hi2c2, acc_addr << 1, 0x43, 1, gyro_data, 6, 2000000);
	int16_t gx_i = (int16_t) ((gyro_data[0] << 8) | gyro_data[1]);
	int16_t gy_i = (int16_t) ((gyro_data[2] << 8) | gyro_data[3]);
	int16_t gz_i = (int16_t) ((gyro_data[4] << 8) | gyro_data[5]);
	float gx = gx_i * (GYRO_FS / (1<<15));
	float gy = gy_i * (GYRO_FS / (1<<15));
	float gz = gz_i * (GYRO_FS / (1<<15));

	uint8_t acc_data[6] = {0x69, 0x69, 0x69, 0x69, 0x69, 0x69};
	HAL_I2C_Mem_Read(&hi2c2, acc_addr << 1, 0x3b, 1, acc_data, 6, 2000000);
	int16_t ax_i = (int16_t) ((acc_data[0] << 8) | acc_data[1]);
	int16_t ay_i = (int16_t) ((acc_data[2] << 8) | acc_data[3]);
	int16_t az_i = (int16_t) ((acc_data[4] << 8) | acc_data[5]);
	float ax = ax_i * (ACC_FS / (1<<15));
	float ay = ay_i * (ACC_FS / (1<<15));
	float az = az_i * (ACC_FS / (1<<15));

	// Compensate for zero point and scale
    float az_mid = (13.45 + -6.7) / 2;
    float az_range = (13.45 - -6.7) / 2;
    float ay_mid = (9.6 + -10.1) / 2;
    float ay_range = (9.6 - -10.1) / 2;

    float az_adj = (az - az_mid) / az_range * 9.82;
    float ay_adj = (ay - ay_mid) / ay_range * 9.82;


	return (AccData) {
		.gx = gx, .gy = gy, .gz = gz,
		.ax = ax, .ay = ay_adj, .az = az_adj
	};
}


struct {
	EscData last_esc;
	AccData last_acc;

	States st;
	Matrix qs;
	float a;
} CTRL;

// #define QUEUE_DEBUG

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char dbgbuf[500];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);

	queue_init(&MAIN_QUEUE);

	char *msg = "\r\n\r\nhewwo hii!!!!\r\n";
	HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 1000000);

	//vesc_init(&huart2, &huart3, &MAIN_QUEUE);
	vesc_init(&huart2, &huart3, &MAIN_QUEUE);

	setup_mpu(&hi2c2, MPU_ADDR);

	// queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_REQ_SENSORS });
	// queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_FLUSH_VESC });
	//queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_REQ_SENSORS });
	//queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_REQ_SENSORS });
	//queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_TIME_STEP });
	//queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_SEND_DEBUG });

	struct {
		float dt;
		float current;
		float wheel_pos_d;
	} dbg_values;

	// For Kalman + control system

	CTRL.a = 0;
	CTRL.st = (States) { .x1 = 0, .x2 = 0, .x3 = 0, .x4 = 0 };
	float init_dt = 0.001;
	CTRL.qs = (Matrix) { .m11 = 0.05 * init_dt * init_dt, .m12 = 0.05 * init_dt * init_dt, .m21 = 0.05 * init_dt * init_dt, .m22 = 0.05 };


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (!queue_has(&MAIN_QUEUE)) {
			continue;
		}

		Message msg = queue_pop(&MAIN_QUEUE);

		switch (msg.ty) {
		case MSG_NONE:
			break;

		case MSG_SEND_DEBUG: {
			// TODO: Send state of kalman filter and controller

			char ctrl_hex[2 * sizeof(CTRL) + 2];
			char *ctrl_data = (void*) &CTRL;
			for (int i = 0; i < sizeof(CTRL); i++) {
				write_hex(&ctrl_hex[2*i], ctrl_data[i]);
			}
			ctrl_hex[2 * sizeof(CTRL)] = 0;

			int dbglen = sprintf(
				dbgbuf,
				"qsz = %4d. t = %8lu us. current = %6ld mA, erpm = %8ld, ax = %8ld, ay = %8ld, az = %8ld. CTRL = \r\n",
				queue_nelem(&MAIN_QUEUE), (int32_t) us_since_startup(), (int32_t) (1000 * dbg_values.current), (int32_t) (1000 * CTRL.last_esc.erpm),
				(int32_t) (1000 * CTRL.last_acc.ax),
				(int32_t) (1000 * CTRL.last_acc.ay),
				(int32_t) (1000 * CTRL.last_acc.az)
			);

			HAL_UART_Transmit_IT(&huart3, (uint8_t *) dbgbuf, dbglen);
			// Use below if you are debug printing other things
			// HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 10000);

			break;
		}

		case MSG_TIME_STEP: {
			uint32_t dt_us = get_and_reset_dt_us();
			float dt = (float) dt_us / 1000000.0;

#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: TIME_STEP by %7.5f}\r\n",
				dt
			);

			HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 10000);
#endif

			dbg_values.dt = dt;

			float wheel_rpm = CTRL.last_esc.erpm / 22.9;
			float wheel_rad_per_second = wheel_rpm * 2 * 3.14 / 60;
			float wheel_pos_d = -wheel_rad_per_second * WHEEL_RAD;

			dbg_values.wheel_pos_d = wheel_pos_d;

			CTRL.qs.m11 = 2 * dt*dt*dt*dt / 4;
			CTRL.qs.m12 = 2 * dt*dt*dt / 4;
			CTRL.qs.m21 = 2 * dt*dt / 4;
			CTRL.qs.m22 = 2 * dt / 4;

			CTRL.st.x4 = wheel_pos_d;
			CTRL.st.x3 += wheel_pos_d * dt;

			pitch_kalman_filter_predict(CTRL.a, dt, &CTRL.st, &CTRL.qs);
			float tau = LookaheadSpeedRegulator(0, CTRL.st.x1, CTRL.st.x2, wheel_pos_d, dt);
			float current = tau / 0.59; // see notes
			current *= 10;

			CTRL.a = sqrt((CTRL.last_acc.ay * CTRL.last_acc.ay) + (CTRL.last_acc.az * CTRL.last_acc.az));
			pitch_kalman_filter_update(CTRL.last_acc.gx, dt, &CTRL.st, &CTRL.qs);

			dbg_values.current = current;

			vesc_set_current(current);

			break;
		}

		case MSG_FLUSH_VESC: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: FLUSH VESC}\r\n"
			);

			HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 10000);
#endif

			vesc_transmit_and_recv(&huart2, &huart3);
			break;
		}

		case MSG_VESC_UART_GOT_DATA: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: UART_GOT_DATA}\r\n"
			);

			HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 10000);
#endif

			vesc_got_data();
			break;
		}

		case MSG_REQ_ACC: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: REQ_ACC}\r\n"
			);

			HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 10000);
#endif
			AccData acc_data = get_accelerometer_data(&hi2c2, MPU_ADDR);
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_GOT_ACC_DATA, .acc_data = acc_data });

			break;

		}

		case MSG_REQ_VESC: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: REQ_VESC}\r\n"
			);

			HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 10000);
#endif

			vesc_request_data();

			break;
		}
		case MSG_GOT_ACC_DATA: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: GOT_ACC_DATA}\r\n"
			);

			HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 10000);
#endif

			AccData acc_data = msg.acc_data;
			CTRL.last_acc = acc_data;

			break;
		}

		case MSG_GOT_ESC_DATA: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: GOT_ESC_DATA temp=%7.5f erpm=%7.5f}\r\n",
				msg.esc_data.temp_mos, msg.esc_data.erpm
			);

			HAL_UART_Transmit(&huart3, (uint8_t *) dbgbuf, dbglen, 100000);
#endif

			EscData esc_data = msg.esc_data;
			CTRL.last_esc = esc_data;

			break;
		}
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10B0DCFB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 96;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9600;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9600;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BTN1_O_GPIO_Port, BTN1_O_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_T_Pin BTN1_I_Pin */
  GPIO_InitStruct.Pin = BTN1_T_Pin|BTN1_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_O_Pin */
  GPIO_InitStruct.Pin = BTN1_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BTN1_O_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

