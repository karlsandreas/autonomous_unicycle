/*
 * vesc_com.h
 *
 *  Created on: 22 feb. 2023
 *      Author: jonathanloov
 */

#ifndef SRC_VESC_COM_H_
#define SRC_VESC_COM_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"
#include "queue.h"

#define RX_DATA_LEN 1000
#define UART_RXSZ 128

typedef struct {
	uint8_t vesc_id;

	uint8_t tx_data[1024];
	size_t current_offset;
	volatile bool tx_waiting;

	uint8_t rx_buf[UART_RXSZ];

	UART_HandleTypeDef *vesc_uart;
	IRQn_Type uart_irq;

	Queue *q;

	volatile uint8_t rx_data[RX_DATA_LEN];
	volatile size_t rx_offset;
	volatile bool rx_queued;

	uint8_t cooldown;
} VESC;

void vesc_init(VESC *vesc, uint8_t vesc_id, UART_HandleTypeDef *vesc_uart, IRQn_Type uart_irq, Queue *q);

// Half of the buffer size, what is sent to the queue

void vesc_uart_cb_txcplt(VESC *vesc, UART_HandleTypeDef *huart);
void vesc_uart_cb_rxcplt(VESC *vesc, UART_HandleTypeDef *huart);

void vesc_start_recv(VESC *vesc);
void vesc_got_data(VESC *vesc);

void vesc_transmit_and_recv(VESC *vesc);
void vesc_set_current(VESC *vesc, float current);
void vesc_request_data(VESC *vesc);

#endif /* SRC_VESC_COM_H_ */
