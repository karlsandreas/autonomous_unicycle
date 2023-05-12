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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "vesc_com.h"
#include "vesc_current_reg.h"
#include "queue.h"
#include "ctrl/common.h"
#include "ctrl/regulator.h"
#include "ctrl/kalman_filter.h"
// #include "buf.h"

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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define TIM_REALTIME htim1
#define TIM_SCHEDULER htim2

#define UART_VESC_PITCH huart2
#define UART_IRQ_VESC_PITCH USART2_IRQn

#define UART_VESC_ROLL huart3
#define UART_IRQ_VESC_ROLL USART3_IRQn

// #define UART_VESC_ROLL huart3
#define I2C_MPU hi2c2

uint16_t MPU_ADDR = 0x68;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void dead_mans_switch_update_led();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static Queue MAIN_QUEUE;

#define VESC_PITCH_ID 1
#define VESC_ROLL_ID 2

static VESC vesc_pitch;
static VESC vesc_roll;

VESC_Current_Reg vcr_pitch = (VESC_Current_Reg) {
	.k_p = 1.,
};
VESC_Current_Reg vcr_roll = (VESC_Current_Reg) {
	.k_p = 0.1,
};

uint32_t ms_counter;

// All in Hz
#define SCHEDULER_FREQ 10000

#define ACC_FREQ 100
#define VESC_FREQ 50
#define DUMP_FREQ 100
#define STEP_FREQ 500

// (will take many hours to overflow)
uint32_t scheduler_ctr = 0;

float controller_lp_tau = 0.3;
float controller_val_pitch = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &TIM_REALTIME) { // TIM_REALTIME ticks once every us, elapses once every ms
		ms_counter++;
		dead_mans_switch_update_led();
	}
	if (htim == &TIM_SCHEDULER ) {
		scheduler_ctr++;

		if (scheduler_ctr % (SCHEDULER_FREQ / ACC_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_REQ_ACC });
		}
		if (scheduler_ctr % (SCHEDULER_FREQ / VESC_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_REQ_VESC });
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_FLUSH_VESC });
		}
		if (scheduler_ctr % (SCHEDULER_FREQ / DUMP_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_SEND_DEBUG });
		}
		if (scheduler_ctr % (SCHEDULER_FREQ / STEP_FREQ) == 0) {
			queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_TIME_STEP });
		}

		float controller_val_instant = HAL_GPIO_ReadPin(CONTROLLER_PWM_GPIO_Port, CONTROLLER_PWM_Pin);
		controller_val_pitch = (1 - 1 / (controller_lp_tau * SCHEDULER_FREQ)) * controller_val_pitch
				+ (1 / (controller_lp_tau * SCHEDULER_FREQ)) * controller_val_instant;
	}
}

float controller_mapped() {
	if (controller_val_pitch < 0.04) {
		return 0;
	}
	return (controller_val_pitch - 0.1) / (0.033333);
}


// This will overflow after 2^32/10^6s ≈ 4300s ≈ 1h11m
// Overflow should be handled by the get_dt and reset_dt
uint32_t us_since_startup() {
	uint32_t us_counter = ms_counter * 1000;
	uint32_t us_timer = TIM_REALTIME.Instance->CNT;
	return us_counter + us_timer;
}

uint32_t last_step = 0;
uint32_t get_and_reset_dt_us() {
	uint32_t now = us_since_startup();
	uint32_t dt = now - last_step; // If now < last_step, then we have overflowed. This should still get the right value
	last_step = now;
	return dt;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &UART_VESC_PITCH) {
		vesc_uart_cb_txcplt(&vesc_pitch, huart);
	}
	if (huart == &UART_VESC_ROLL) {
		vesc_uart_cb_txcplt(&vesc_roll, huart);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &UART_VESC_PITCH) {
		vesc_uart_cb_rxcplt(&vesc_pitch, huart);
	}
	if (huart == &UART_VESC_ROLL) {
		vesc_uart_cb_rxcplt(&vesc_roll, huart);
	}
}

int uart_error_count_pitch = 0;
int uart_error_count_roll = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == &UART_VESC_PITCH) {
		uart_error_count_pitch++;
		vesc_pitch.cooldown = 3;

		HAL_UART_Abort_IT(&UART_VESC_PITCH);
		__HAL_UART_CLEAR_OREFLAG(&UART_VESC_PITCH);

		vesc_pitch.tx_waiting = false;
		vesc_pitch.rx_queued = false;
	}
	if (huart == &UART_VESC_ROLL) {
		uart_error_count_roll++;
		vesc_roll.cooldown = 3;

		HAL_UART_Abort_IT(&UART_VESC_ROLL);
		__HAL_UART_CLEAR_OREFLAG(&UART_VESC_ROLL);

		vesc_roll.tx_waiting = false;
		vesc_roll.rx_queued = false;
	}
	// abort_vesc();
}

void abort_vesc() {
	HAL_UART_Abort_IT(&UART_VESC_PITCH);
	vesc_pitch.tx_waiting = false;
	vesc_pitch.rx_queued = false;

	HAL_UART_Abort_IT(&UART_VESC_ROLL);
	vesc_roll.tx_waiting = false;
	vesc_roll.rx_queued = false;
/*
	vesc_start_recv(&vesc_pitch);
	vesc_start_recv(&vesc_roll);
*/
}

bool dead_mans_switch_activated() {
	HAL_GPIO_WritePin(DEADMAN_GND_GPIO_Port, DEADMAN_GND_Pin, GPIO_PIN_RESET);
	return HAL_GPIO_ReadPin(DEADMAN_SW_GPIO_Port, DEADMAN_SW_Pin) == GPIO_PIN_RESET;
}

// QRV
const bool morse_table[] = {
	1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0,
	1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0,
	1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0,
	0, 0, 0, 0, 0
};

void dead_mans_switch_update_led() {
	// bool blink_on = (ms_counter % 600 < 300) && ((ms_counter & 3) == 1);
	bool blink_on = morse_table[(ms_counter / 83) % sizeof(morse_table)];
	blink_on &= (ms_counter & 3) == 1;
	blink_on |= dead_mans_switch_activated();

	HAL_GPIO_WritePin(DEADMAN_LED_GPIO_Port, DEADMAN_LED_Pin, blink_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#define I2C_TIMEOUT 20

#define ACC_SANITY 40

// Puts the MPU6050 into active mode, with the appropriate CFGs from main.h set
void setup_mpu(I2C_HandleTypeDef *i2c, uint16_t acc_addr) {
	uint8_t pwr_mgnt_data[1] = {0x00}; // sleep = 0, cycle = 0
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x6b, 1, pwr_mgnt_data, 1, I2C_TIMEOUT); // Disable sleep

	uint8_t config[1] = {(0x0 << 3) | DLPF_CFG}; // fsync = 0, dlpf = 6 (5Hz)
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x1a, 1, config, 1, I2C_TIMEOUT);

	uint8_t gyro_config[1] = {(0x0 << 5) | (GYRO_FS_CFG << 3)}; // no self-test
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x1b, 1, gyro_config, 1, I2C_TIMEOUT);

	uint8_t acc_config[1] = {(0x0 << 5) | (ACC_FS_CFG << 3)}; // no self-test
	HAL_I2C_Mem_Write(i2c, acc_addr << 1, 0x1c, 1, acc_config, 1, I2C_TIMEOUT);
}

// TODO: Do this with interrupt
AccData get_accelerometer_data(I2C_HandleTypeDef *i2c, uint16_t acc_addr) {
	// 0x43 = gyro_xout_h
	// 0x44 = gyro_xout_
	uint8_t gyro_data[6] = {0x69, 0x69, 0x69, 0x69, 0x69, 0x69};

	if (HAL_I2C_Mem_Read(&hi2c2, acc_addr << 1, 0x43, 1, gyro_data, 6, I2C_TIMEOUT) == HAL_ERROR) {
		return (AccData) { .success = false };
	}

	int16_t gx_i = (int16_t) ((gyro_data[0] << 8) | gyro_data[1]);
	int16_t gy_i = (int16_t) ((gyro_data[2] << 8) | gyro_data[3]);
	int16_t gz_i = (int16_t) ((gyro_data[4] << 8) | gyro_data[5]);
	float gx = gx_i * (GYRO_FS / (1<<15));
	float gy = gy_i * (GYRO_FS / (1<<15));
	float gz = gz_i * (GYRO_FS / (1<<15));

	uint8_t acc_data[6] = {0x69, 0x69, 0x69, 0x69, 0x69, 0x69};
	if (HAL_I2C_Mem_Read(&hi2c2, acc_addr << 1, 0x3b, 1, acc_data, 6, I2C_TIMEOUT) == HAL_ERROR) {
		return (AccData) { .success = false };
	}
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

    // Calibration constants
    gx += 0.034;
    gy -= 0.023;
    gz -= 0.002;

	return (AccData) {
		.success = true,
		.gx = gx, .gy = gy, .gz = gz,
		.ax = ax, .ay = ay, .az = az
	};
}


struct {
	EscData last_esc_pitch;
	EscData last_esc_roll;

	AccData last_acc;

	States st;
	Matrix q_w, q_t;
	Covariances covs;
	R_error r_vals;

} CTRL;

// For complementary filter
float roll_angle = 0.;
float pitch_angle = 0.;

float complementary_filter_gain = 1.;

PitchRegulator pitch_reg = (PitchRegulator) {
	.setpoint_speed = 0.,
	.setpoint_theta_0 = -0.030,
	.kp1 = 30,
	.kd1 = 8,

	.kp2 = 0.2,
};

RollRegulator roll_reg = (RollRegulator) {
	.setpoint_theta_0 = -0.010,

	.kp1 = -80,
	.kd1 = -70,

	.kp2 = 0.0006,
};

#define MOTOR_CW 0
#define MOTOR_CCW 1

#define MOTOR_DIRECTION_PITCH MOTOR_CCW
#define MOTOR_DIRECTION_ROLL MOTOR_CW

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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
	HAL_Delay(500);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
	HAL_Delay(500);
    HAL_GPIO_WritePin(LDERROR_GPIO_Port, LDERROR_Pin, SET);
	HAL_Delay(500);
  	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
	HAL_Delay(500);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
  	HAL_Delay(500);
    HAL_GPIO_WritePin(LDERROR_GPIO_Port, LDERROR_Pin, RESET);

  	char *msg = "\r\nhiiii we are started :3\r\n\r\n";
	CDC_Transmit_FS((uint8_t *) msg, strlen(msg));

	queue_init(&MAIN_QUEUE);

	HAL_TIM_Base_Start_IT(&TIM_REALTIME);
	HAL_TIM_Base_Start_IT(&TIM_SCHEDULER);

	vesc_init(&vesc_pitch, VESC_PITCH_ID, &UART_VESC_PITCH, UART_IRQ_VESC_PITCH, &MAIN_QUEUE);
	vesc_init(&vesc_roll, VESC_ROLL_ID, &UART_VESC_ROLL, UART_IRQ_VESC_ROLL, &MAIN_QUEUE);

	setup_mpu(&I2C_MPU, MPU_ADDR);

	struct {
		float dt;
		float current_wanted_pitch, current_out_pitch;
		float current_wanted_roll, current_out_roll;
		float setpoint_theta;

		float wheel_pos_d;
		float pitch_rpm, roll_rpm;
		unsigned int msgs_since_last;

		int msg_idx, n_time_steps_since_last;
	} dbg_values;

	dbg_values.msgs_since_last = 0;
	dbg_values.msg_idx = 0;
	dbg_values.n_time_steps_since_last = 0;


	// For Kalman + control system

	CTRL.covs = (Covariances) {.pitch.m11 = 10.0, .pitch.m12 = 0.0, .pitch.m21 = 10.0, .pitch.m22 = 0.0,
	                      .roll.m11 = 10.0, .roll.m12 = 0.0, .roll.m21 = 10.0, .roll.m22 = 0.0,
	                      .wheel.m11 = 10.0, .wheel.m12 = 0.0, .wheel.m21 = 0.0, .wheel.m22 = 10.0};


	//Measurement error
	CTRL.r_vals = (R_error) {.pitch = 0.3, .roll = 0.3, .wheel = 20};

	CTRL.st = (States) { .x1 = 0, .x2 = 0, .x3 = 0, .x4 = 0 };


	queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_SEND_DEBUG });

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
		dbg_values.msgs_since_last++;

		switch (msg.ty) {
		case MSG_NONE:
			break;

		case MSG_SEND_DEBUG: {
			// TODO: Send state of kalman filter and controller

			/*
			char ctrl_hex[2 * sizeof(CTRL) + 2];
			char *ctrl_data = (void*) &CTRL;
			for (int i = 0; i < sizeof(CTRL); i++) {
				write_hex(&ctrl_hex[2*i], ctrl_data[i]);
			}
			ctrl_hex[2 * sizeof(CTRL)] = 0;
			*/

			bool dead_mans = dead_mans_switch_activated();

			int dbglen = sprintf(
				dbgbuf,
				"msg = %4d, nerr = %4dp/%4dr, time steps = %4d, qsz = %4d, switch = %s, t = %8lu ms, "
				//"kp1 = %7.4f, kd1 = %7.4f, kp2 = %7.4f, setpoint_theta_0 = %7.4f, "
				"ax = %7.4f, ay = %7.4f, az = %7.4f, "
				"gx = %7.4f rad/s, gy = %7.4f rad/s, gz = %7.4f rad/s, "
				"rpm_pitch = %7.4f, rpm_roll = %7.4f, "
				"I_w_pitch = %7.4f A, I_w_roll = %7.4f A, "
				"theta_pitch = %7.5fmrad, "
				"theta_roll_comp = %7.4f mrad, "
				"theta_setpoint = %7.4f mrad, "
				"ctrl duty = %7.4f%%, ctrl amount = %7.4f "
				//"I (filtered) = %6ld mA, I (out) = %6ld mA"
				"\r\n",
				dbg_values.msg_idx, uart_error_count_pitch, uart_error_count_roll, dbg_values.n_time_steps_since_last, queue_nelem(&MAIN_QUEUE), dead_mans ? "on" : "off", (int32_t) (us_since_startup() / 1000),
				//roll_reg.kp1, roll_reg.kd1, roll_reg.kp2, roll_reg.setpoint_theta_0,
				CTRL.last_acc.ax, CTRL.last_acc.ay, CTRL.last_acc.az,
				CTRL.last_acc.gx, CTRL.last_acc.gy, CTRL.last_acc.gz,
				dbg_values.pitch_rpm, dbg_values.roll_rpm,
				dbg_values.current_wanted_pitch, dbg_values.current_wanted_roll,
				1000 * pitch_angle,
				// 1000 * CTRL.st.x5, 1000 * CTRL.st.x6
				1000 * roll_angle,
				1000 * dbg_values.setpoint_theta,
				100 * controller_val_pitch, controller_mapped()
				//(int32_t) (1000 * vcr.input_filtered), (int32_t) (1000 * dbg_values.current_o)
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);


			dbg_values.msg_idx++;
			dbg_values.n_time_steps_since_last = 0;


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

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			dbg_values.dt = dt;

#if MOTOR_DIRECTION_PITCH == MOTOR_CW
			float wheel_rpm_pitch = CTRL.last_esc_pitch.erpm / -22.9;
#elif MOTOR_DIRECTION_PITCH == MOTOR_CCW
			float wheel_rpm_pitch = CTRL.last_esc_pitch.erpm / 22.9;
#else
#error "Invalid motor direction"
#endif

#if MOTOR_DIRECTION_ROLL == MOTOR_CW
			float wheel_rpm_roll = CTRL.last_esc_roll.erpm / -29.92;
#elif MOTOR_DIRECTION_ROLL == MOTOR_CCW
			float wheel_rpm_roll = CTRL.last_esc_roll.erpm / 29.92;
#else
#error "Invalid motor direction"
#endif

			dbg_values.pitch_rpm = wheel_rpm_pitch;
			dbg_values.roll_rpm = wheel_rpm_roll;

			float ground_speed_pitch = wheel_rpm_pitch / 60 * 6.28 * 0.28;


			float sensor_gyro_pitch = CTRL.last_acc.gy;
			float sensor_gyro_roll = CTRL.last_acc.gx;

			float acc_predicted_angle_pitch = -atan2(CTRL.last_acc.ax, CTRL.last_acc.az);
			float acc_predicted_angle_roll = atan2(CTRL.last_acc.ay, CTRL.last_acc.az);

			// Sanity check: sometimes the accelerometer gives bogus values, we shouldn't ever go over 1 radian in angle
			if (fabs(acc_predicted_angle_pitch) > 1 || fabs(acc_predicted_angle_roll) > 1) {
				acc_predicted_angle_pitch = pitch_angle;
				acc_predicted_angle_roll = roll_angle;
			}

			float dgain = complementary_filter_gain * dt;
			roll_angle = (1 - dgain) * roll_angle + dt * sensor_gyro_roll + dgain * acc_predicted_angle_roll;
			pitch_angle = (1 - dgain) * pitch_angle + dt * sensor_gyro_pitch + dgain * acc_predicted_angle_pitch;

			dbg_values.setpoint_theta = roll_reg_setpoint_theta(&roll_reg, dt, roll_angle, sensor_gyro_roll, wheel_rpm_roll);

			pitch_reg.setpoint_speed = controller_mapped() * 0.2;

			float tau_pitch = pitch_reg_step(&pitch_reg, dt, pitch_angle, sensor_gyro_pitch, ground_speed_pitch);
			float tau_roll = roll_reg_step(&roll_reg, dt, roll_angle, sensor_gyro_roll, wheel_rpm_roll);

			float current_wanted_pitch = tau_pitch / 0.2;
			float current_wanted_roll = tau_roll / 0.2;


			dbg_values.current_wanted_pitch = current_wanted_pitch;
			dbg_values.current_wanted_roll = current_wanted_roll;

			float current_out_pitch, current_out_roll;
			if (dead_mans_switch_activated()) {
				vcr_pitch.setpoint = current_wanted_pitch;
				vcr_roll.setpoint = current_wanted_roll;

				current_out_pitch = vcr_step(&vcr_pitch, dt, CTRL.last_esc_pitch.current_motor);
				current_out_roll = vcr_step(&vcr_roll, dt, CTRL.last_esc_roll.current_motor);
			} else {
				vcr_pitch.input_filtered = 0;
				vcr_pitch.setpoint = 0;
				vcr_roll.input_filtered = 0;
				vcr_roll.setpoint = 0;

				current_out_pitch = 0;
			}


#if MOTOR_DIRECTION_PITCH == MOTOR_CW
			vesc_set_current(&vesc_pitch, -current_out_pitch);
#elif MOTOR_DIRECTION_PITCH == MOTOR_CCW
			vesc_set_current(&vesc_pitch, current_out_pitch);
#else
#error "Invalid motor direction"
#endif

#if MOTOR_DIRECTION_ROLL == MOTOR_CW
			vesc_set_current(&vesc_roll, -current_out_roll);
#elif MOTOR_DIRECTION_ROLL == MOTOR_CCW
			vesc_set_current(&vesc_roll, current_out_roll);
#else
#error "Invalid motor direction"
#endif



			dbg_values.n_time_steps_since_last++;

			break;
		}

		case MSG_FLUSH_VESC: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: FLUSH VESC}\r\n"
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			vesc_transmit_and_recv(&vesc_pitch);
			vesc_transmit_and_recv(&vesc_roll);
			break;
		}

		case MSG_VESC_UART_GOT_DATA: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: UART_GOT_DATA}\r\n"
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			if (msg.vesc_id == VESC_PITCH_ID) {
				vesc_got_data(&vesc_pitch);
			}
			if (msg.vesc_id == VESC_ROLL_ID) {
				vesc_got_data(&vesc_roll);
			}
			break;
		}

		case MSG_REQ_ACC: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: REQ_ACC}\r\n"
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif
			AccData acc_data = get_accelerometer_data(&hi2c2, MPU_ADDR);
			if (acc_data.success == false) {
				int dbglen = sprintf(
					dbgbuf,
					"[ACC I2C FAIL]\r\n"
				);

				CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
			} else {
				queue_put(&MAIN_QUEUE, (Message) { .ty = MSG_GOT_ACC_DATA, .acc_data = acc_data });
			}

			break;

		}

		case MSG_REQ_VESC: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: REQ_VESC}\r\n"
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			vesc_request_data(&vesc_pitch);
			vesc_request_data(&vesc_roll);

			break;
		}
		case MSG_GOT_ACC_DATA: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: GOT_ACC_DATA}\r\n"
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			AccData acc_data = msg.acc_data;
			if (fabs(acc_data.ax) < ACC_SANITY && fabs(acc_data.ay) < ACC_SANITY && fabs(acc_data.az) < ACC_SANITY) {
				CTRL.last_acc = acc_data;
			}

			break;
		}

		case MSG_GOT_ESC_DATA: {
#ifdef QUEUE_DEBUG
			int dbglen = sprintf(
				dbgbuf,
				"{Q: GOT_ESC_DATA temp=%7.5f erpm=%7.5f}\r\n",
				msg.esc_data.temp_mos, msg.esc_data.erpm
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			EscData esc_data = msg.esc_data;
			if (esc_data.vesc_id == VESC_PITCH_ID) {
				CTRL.last_esc_pitch = esc_data;
			}
			if (esc_data.vesc_id == VESC_ROLL_ID) {
				CTRL.last_esc_roll = esc_data;
			}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED3_Pin|LDERROR_Pin|DEADMAN_GND_Pin
                          |DEADMAN_LED_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED3_Pin LDERROR_Pin DEADMAN_GND_Pin
                           DEADMAN_LED_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|LDERROR_Pin|DEADMAN_GND_Pin
                          |DEADMAN_LED_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DEADMAN_SW_Pin */
  GPIO_InitStruct.Pin = DEADMAN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DEADMAN_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CONTROLLER_PWM_Pin */
  GPIO_InitStruct.Pin = CONTROLLER_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(CONTROLLER_PWM_GPIO_Port, &GPIO_InitStruct);

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

