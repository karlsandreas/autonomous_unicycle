/*
 * queue.c
 *
 *  Created on: Mar 2, 2023
 *      Author: jonathanloov
 */


#include "queue.h"
#include "cmsis_gcc.h"

void queue_init(Queue *q) {
	q->read_idx = 0;
	q->write_idx = 0;
	q->is_reading = false;
	q->is_writing = false;
}

size_t queue_nelem(Queue *q) {
	return (q->write_idx - q->read_idx) % CHANNEL_SIZE;
}

bool queue_can_put(Queue *q) {
	return queue_nelem(q) < CHANNEL_SIZE - 1;
}

// Returns false if value cannot be put (queue is full)
bool queue_put(Queue *q, Message msg) {
	if (!queue_can_put(q)) {
		return false;
	}

	while (q->is_writing) { }
	__disable_irq();

	q->is_writing = true;

	q->messages[q->write_idx] = msg;
	q->write_idx = (q->write_idx + 1) % CHANNEL_SIZE;

	q->is_writing = false;
	__enable_irq();

	return true;
}

bool queue_has(Queue *q) {
	return queue_nelem(q) != 0;
}

// Returns a MSG_NONE if the queue is empty
Message queue_pop(Queue *q) {
	if (!queue_has(q)) {
		return (Message) { .ty = MSG_NONE };
	}
	while (q->is_reading) { }

	__disable_irq();
	q->is_reading = true;

	Message msg = q->messages[q->read_idx];
	q->read_idx = (q->read_idx + 1) % CHANNEL_SIZE;

	q->is_reading = false;
	__enable_irq();

	return msg;
}
