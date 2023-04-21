/*
 * vesc_com.h
 *
 *  Created on: 22 feb. 2023
 *      Author: jonathanloov
 */

#ifndef SRC_VESC_COM_H_
#define SRC_VESC_COM_H_

#include<stdint.h>

#include "stm32f4xx_hal.h"
#include "queue.h"

// Half of the buffer size, what is sent to the queue

void vesc_uart_cb_txcplt(UART_HandleTypeDef *huart);
void vesc_uart_cb_rxcplt(UART_HandleTypeDef *huart);

void vesc_start_recv();
void vesc_got_data();

void vesc_init(UART_HandleTypeDef *vesc_uart, Queue *q);
void vesc_transmit_and_recv();
void vesc_set_current(float current);
void vesc_request_data();

#endif /* SRC_VESC_COM_H_ */
