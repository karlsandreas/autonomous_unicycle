/*
 * vesc_com.h
 *
 *  Created on: 22 feb. 2023
 *      Author: jonathanloov
 */

#ifndef SRC_VESC_COM_H_
#define SRC_VESC_COM_H_

#include<stdint.h>

#include "stm32h7xx_hal.h"

void vesc_uart_cb_txcplt();
void vesc_queue_packet(uint8_t *content, size_t len);
void vesc_transmit(UART_HandleTypeDef *huart, UART_HandleTypeDef *dbghuart);
void vesc_set_current(float current);
void vesc_request_data();

#endif /* SRC_VESC_COM_H_ */
