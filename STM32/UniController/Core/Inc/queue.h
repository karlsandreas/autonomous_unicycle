/*
 * channel.h
 *
 *  Created on: Mar 2, 2023
 *      Author: jonathanloov
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define CHANNEL_SIZE 1024

#define CLEAR_MESSAGES false

typedef struct {
	bool success;

	uint8_t acc_id;
	float gx, gy, gz;
	float ax, ay, az;
} AccData;

typedef struct {
	float temp_mos;
	float erpm;
	float current_motor;
} EscData;

typedef struct {
	enum {
		MSG_NONE = 0, // "Null" message. Cannot be put in the queue. Will only be returned
		MSG_SEND_DEBUG, // Send state data over UART3 (USB)

		MSG_TIME_STEP, // Update kalman filter, run control system, send current to ESC. Flushes all messages to the ESC

		MSG_REQ_VESC,
		MSG_FLUSH_VESC, // Send all data to VESC
		MSG_REQ_ACC,

		MSG_VESC_UART_GOT_DATA,
		MSG_GOT_ACC_DATA,
		MSG_GOT_ESC_DATA,
	} ty;
	union {
		AccData acc_data;

		EscData esc_data;
	};
} Message;


typedef struct {
	Message messages[CHANNEL_SIZE];
	volatile size_t read_idx; // points to the index of the next message to read
	volatile size_t write_idx; // points to the index where the next message will be written

	volatile bool is_writing, is_reading;
} Queue;

void queue_init(Queue *q);
size_t queue_nelem(Queue *q);

bool queue_can_put(Queue *q);

// Returns false if value cannot be put (queue is ful)
bool queue_put(Queue *q, Message msg);

bool queue_has(Queue *q);

// Returns a MSG_NONE if the queue is empty
Message queue_pop(Queue *q);

#endif /* INC_QUEUE_H_ */
