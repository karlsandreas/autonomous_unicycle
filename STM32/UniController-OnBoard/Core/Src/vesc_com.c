/*
 * vesc_com.c
 *
 *  Created on: 22 feb. 2023
 *      Author: jonathanloov
 */

#include "vesc_com.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "main.h" // for gpio pins
#include "buf.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

_Bool dead_mans_switch_activated(); // prototype from main.c

// From https://github.com/vedderb/bldc_uart_comm_stm32f4_discovery
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING
} COMM_PACKET_ID;

const uint16_t crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
		0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
		0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
		0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
		0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
		0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
		0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
		0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
		0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
		0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
		0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
		0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
		0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
		0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
		0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
		0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
		0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
		0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
		0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
		0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
		0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
		0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
		0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
		0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
		0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

#define RX_DATA_LEN 1000
#define UART_RXSZ 128

//#define DEBUG_COMM 1
//#define DEBUG_VERBOSE 1

static struct {
	uint8_t tx_data[1024];
	size_t current_offset;
	volatile bool tx_waiting;

	uint8_t rx_buf[UART_RXSZ];

	UART_HandleTypeDef *vesc_uart;

	Queue *q;

	volatile uint8_t rx_data[RX_DATA_LEN];
	volatile size_t rx_offset;
} VESC;

static inline uint8_t read_u8(uint8_t **data) {
	uint8_t val = **data;
	(*data)++;
	return val;
}

static inline uint16_t read_u16(uint8_t **data) {
	uint8_t hi = read_u8(data);
	uint8_t lo = read_u8(data);
	return (uint16_t) lo | ((uint16_t) hi << 8);
}

static inline uint32_t read_u32(uint8_t **data) {
	uint16_t hi = read_u16(data);
	uint16_t lo = read_u16(data);
	return (uint32_t) lo | ((uint32_t) hi << 16);
}

static inline float read_f16(uint8_t **data, uint16_t scale) {
	uint16_t uval = read_u16(data);
	int16_t val = uval < INT16_MAX ? uval : (int16_t) (uval - INT16_MAX) - INT16_MAX;

	return (float) val / (float) scale;
}

static inline float read_f32(uint8_t **data, uint32_t scale) {
	uint32_t uval = read_u32(data);
	int32_t val = uval < INT32_MAX ? uval : (int32_t) (uval - INT32_MAX) - INT32_MAX;

	return (float) val / (float) scale;
}


void vesc_init(UART_HandleTypeDef *vesc_uart, Queue *q) {
	VESC.vesc_uart = vesc_uart;

	VESC.tx_waiting = 0;

	VESC.q = q;

	vesc_start_recv();
}

void vesc_uart_cb_txcplt(UART_HandleTypeDef *huart) {

#ifdef DEBUG_VERBOSE
	char *msg = "[TX DONE]\r\n";
	CDC_Transmit_FS((uint8_t *) msg, strlen(msg));
#endif

	VESC.tx_waiting = false;
}

static volatile bool rx_queued = false;

void vesc_start_recv() {
	HAL_UART_Receive_IT(VESC.vesc_uart, VESC.rx_buf, UART_RXSZ);
}

void vesc_uart_cb_rxcplt(UART_HandleTypeDef *_huart) {
	if (VESC.rx_offset + UART_RXSZ < RX_DATA_LEN) {
		// TODO: handle if we are out of bounds?
		// memcpy(VESC.rx_data + VESC.rx_offset, VESC.rx_buf, UART_RXSZ);
		for (int i = 0; i < UART_RXSZ; i++) {
			VESC.rx_data[VESC.rx_offset + i] = VESC.rx_buf[i];
		}
		VESC.rx_offset += UART_RXSZ;
	}

	if (!rx_queued) {
		queue_put(VESC.q, (Message) { .ty = MSG_VESC_UART_GOT_DATA });
		rx_queued = true;
	}

	vesc_start_recv();
}

void vesc_got_data() {
	rx_queued = false;

	if (VESC.rx_offset == 0) {
#ifdef DEBUG_VERBOSE
		char *no_data = "<RX: NO DATA>\r\n";
		CDC_Transmit_FS((uint8_t*) no_data, strlen(no_data));
#endif
		return;
	}

#ifdef DEBUG_COMM

	char hexbuf[RX_DATA_LEN * 2];
	for (int i = 0; i < VESC.rx_offset; i++) {
		write_hex(hexbuf + 2 * i, VESC.rx_data[i]);
	}
	hexbuf[2 * VESC.rx_offset] = 0;

	char dbgbuf[RX_DATA_LEN * 2 + 100];
	int dbglen = snprintf(
		dbgbuf, sizeof(dbgbuf),
		"[RX %d: %s]\r\n",
		VESC.rx_offset, hexbuf
	);

	CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

	// parse message
	size_t offset = 0;
	while (offset < VESC.rx_offset) {
		if (VESC.rx_data[offset] != 2) {
			// Invalid message found!

#ifdef DEBUG_COMM
			int dbglen = snprintf(
				dbgbuf, sizeof(dbgbuf),
				"<PARSE ERR @%d/%d: Invalid header>\r\n",
				offset, VESC.rx_offset
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif
			offset++;
			continue;
		}

		uint8_t msg_size = VESC.rx_data[offset + 1];

		if (offset + msg_size + 5 > VESC.rx_offset) {
#ifdef DEBUG_COMM

			int dbglen = snprintf(
				dbgbuf, sizeof(dbgbuf),
				"<PARSE STOP @%d/%d: len %d too long>\r\n",
				offset, VESC.rx_offset, msg_size
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			break;
		}

		if (VESC.rx_data[offset + msg_size + 4] != 0x3) {

#ifdef DEBUG_COMM
			int dbglen = snprintf(
				dbgbuf, sizeof(dbgbuf),
				"<PARSE ERR @%d/%d: Invalid trailer>\r\n",
				offset, VESC.rx_offset
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif


			offset++;
			continue;
		}

		// TODO: Check crc

#ifdef DEBUG_COMM
		for (int i = 0; i < msg_size; i++) {
			write_hex(hexbuf + 2 * i, VESC.rx_data[offset + 2 + i]);
		}
		hexbuf[2 * msg_size] = 0;

		int dbglen = snprintf(
			dbgbuf, sizeof(dbgbuf),
			"<PARSE DATA @%d/%d: %d bytes: %s>\r\n",
			offset, VESC.rx_offset, msg_size, hexbuf
		);

		CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

		uint8_t *packet = &VESC.rx_data[offset + 2];

		uint8_t msg_type = packet[0];
		switch (msg_type) {
		case COMM_GET_VALUES: {
			uint8_t *pkt_ptr = &packet[1];

			float temp_mos = read_f16(&pkt_ptr, 1e1);
			/*float temp_motor = */read_f16(&pkt_ptr, 1e1);
			float current_motor = read_f32(&pkt_ptr, 1e2);
			/*float current_in = */read_f32(&pkt_ptr, 1e2);
			/*float id = */read_f32(&pkt_ptr, 1e2);
			/*float iq = */read_f32(&pkt_ptr, 1e2);
			/*float duty_now = */read_f16(&pkt_ptr, 1e3);
			float rpm = read_f32(&pkt_ptr, 1e0);
			/*float v_in = */read_f16(&pkt_ptr, 1e1);
			/*float amp_hours = */read_f32(&pkt_ptr, 1e4);
			/*float amp_hours_charged = */read_f32(&pkt_ptr, 1e4);
			/*float watt_hours = */read_f32(&pkt_ptr, 1e4);
			/*float watt_hours_charged = */read_f32(&pkt_ptr, 1e4);

			Message msg = (Message) { .ty = MSG_GOT_ESC_DATA, .esc_data = { .temp_mos = temp_mos, .erpm = rpm, .current_motor = current_motor } };

			queue_put(VESC.q, msg);
#ifdef DEBUG_COMM
			int dbglen = snprintf(
				dbgbuf, sizeof(dbgbuf),
				//"<VALUES: temp_mos = %7.5f, temp_motor = %7.5f, current_motor = %7.5f, current_in = %7.5f, id = %7.5f, iq = %7.5f, duty_now = %7.5f, rpm = %7.5f, v_in = %7.5f, amp_hours = %7.5f, amp_hours_charged = %7.5f, watt_hours = %7.5f, watt_hours_charged = %7.5f>\r\n",
				//temp_mos, temp_motor, current_motor, current_in, id, iq, duty_now, rpm, v_in, amp_hours, amp_hours_charged, watt_hours, watt_hours_charged
				"<VALUES: temp_mos = %7.5f, current_motor = %7.5f, rpm = %7.5f>\r\n",
				temp_mos, current_motor, rpm
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

			break;
		}
		default: {
			break;

#ifdef DEBUG_COMM
			int dbglen = snprintf(
				dbgbuf, sizeof(dbgbuf),
				"<PARSE PACKET %d/%d: UNKNOWN MESSAGE TYPE 0x%02x>\r\n",
				offset, VESC.rx_offset, msg_type
			);

			CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif
			break;
		}
		}

		offset += 5 + msg_size;
	}

	HAL_NVIC_DisableIRQ(USART2_IRQn);

	// Go back
	memmove((void*) VESC.rx_data, (void*) &VESC.rx_data[offset], VESC.rx_offset - offset);
	VESC.rx_offset = VESC.rx_offset - offset;

	HAL_NVIC_EnableIRQ(USART2_IRQn);

#ifdef DEBUG_VERBOSE
	dbglen = snprintf(
		dbgbuf, sizeof(dbgbuf),
		"<RX DONE>\r\n"
	);

	CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif
}

// If response_size == 0, then we don't expected a response.
void vesc_queue_packet(uint8_t *content, size_t len, size_t response_size) {
	if (len > 256) {
		// TODO: Not yet implemented
		return;
	}

	VESC.tx_data[VESC.current_offset++] = 0x2; // short (<256 bytes) packet
	VESC.tx_data[VESC.current_offset++] = (uint8_t) (len & 0xff);

	uint16_t crc16 = 0;
	for (size_t i = 0; i < len; i++) {
		VESC.tx_data[VESC.current_offset++] = content[i];
		crc16 = (crc16 << 8) ^ crc16_tab[0xff & ((crc16 >> 8) ^ content[i])];
	}
	VESC.tx_data[VESC.current_offset++] = (uint8_t) ((crc16 >> 8) & 0xff);
	VESC.tx_data[VESC.current_offset++] = (uint8_t) (crc16 & 0xff);
	VESC.tx_data[VESC.current_offset++] = 0x3;

}

void vesc_transmit_and_recv() {
	if (VESC.current_offset == 0) {
		return;
	}
	if (VESC.tx_waiting) {

		char *msg = "BLOCK_TX_WAITING\r\n";
		CDC_Transmit_FS((uint8_t*) msg, strlen(msg));

		HAL_GPIO_WritePin(LDERROR_GPIO_Port, LDERROR_Pin, GPIO_PIN_SET);

		while (VESC.tx_waiting) {
			__asm("nop");
		}
		HAL_GPIO_WritePin(LDERROR_GPIO_Port, LDERROR_Pin, GPIO_PIN_RESET);
	}

#ifdef DEBUG_COMM
	char dbgbuf[100];
	int dbglen = snprintf(
		dbgbuf, sizeof(dbgbuf),
		"[TX %d]\r\n",
		VESC.current_offset
	);
	CDC_Transmit_FS((uint8_t *) dbgbuf, dbglen);
#endif

	VESC.tx_waiting = true;
	HAL_UART_Transmit_IT(VESC.vesc_uart, VESC.tx_data, VESC.current_offset);
	VESC.current_offset = 0;

	vesc_start_recv();
}

void vesc_set_current(float current) {
	if (!dead_mans_switch_activated()) {
		current = 0.0;
	}

	uint8_t buf[5];
	buf[0] = COMM_SET_CURRENT;
	int32_t current_i = (int32_t) (current * 1000.0);
	// copy to big endian
	buf[1] = (current_i >> 24) & 0xff;
	buf[2] = (current_i >> 16) & 0xff;
	buf[3] = (current_i >> 8) & 0xff;
	buf[4] = (current_i >> 0) & 0xff;

#ifdef DEBUG_COMM
	char *msg = "[QUEUE setting current]\r\n";
	CDC_Transmit_FS((uint8_t*) msg, strlen(msg));
#endif

	vesc_queue_packet(buf, 5, 0);
}

void vesc_request_data() {
	uint8_t buf[1];
	buf[0] = COMM_GET_VALUES;

#ifdef DEBUG_COMM
	char *msg = "[QUEUE requesting data]\r\n";
	CDC_Transmit_FS((uint8_t*) msg, strlen(msg));
#endif

	vesc_queue_packet(buf, 1, 0x29);
}
