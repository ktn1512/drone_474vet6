/*
 * e22.c
 *
 *  Created on: 23 thg 8, 2026
 *      Author: khanh
 */

#include "e22.h"
#include <string.h>

void E22_Init(E22_t *e22, UART_HandleTypeDef *huart) {
	e22->huart = huart;

	e22->rx_head = 0;
	e22->rx_tail = 0;

	HAL_UART_Receive_IT(e22->huart, &e22->rx_byte, 1);
}

HAL_StatusTypeDef E22_Send(E22_t *e22, const uint8_t *data, uint16_t len) {
	return HAL_UART_Transmit(e22->huart, (uint8_t*) data, len, 100);
}

HAL_StatusTypeDef E22_SendString(E22_t *e22, const char *str) {
	return E22_Send(e22, (uint8_t*) str, strlen(str));
}

void E22_RxCallback(E22_t *e22) {
	uint16_t next = (e22->rx_head + 1) % E22_RX_BUFFER_SIZE;

	if (next != e22->rx_tail) {
		e22->rx_buffer[e22->rx_head] = e22->rx_byte;

		e22->rx_head = next;
	}

	HAL_UART_Receive_IT(e22->huart, &e22->rx_byte, 1);
}

uint16_t E22_Available(E22_t *e22) {
	if (e22->rx_head >= e22->rx_tail) {
		return e22->rx_head - e22->rx_tail;
	}

	return E22_RX_BUFFER_SIZE - e22->rx_tail + e22->rx_head;
}

uint8_t E22_ReadByte(E22_t *e22) {
	uint8_t data = 0;

	if (e22->rx_head != e22->rx_tail) {
		data = e22->rx_buffer[e22->rx_tail];

		e22->rx_tail = (e22->rx_tail + 1) % E22_RX_BUFFER_SIZE;
	}

	return data;
}

uint16_t E22_Read(E22_t *e22, uint8_t *buf, uint16_t max_len) {
	uint16_t count = 0;

	while (E22_Available(e22) && count < max_len) {
		buf[count++] = E22_ReadByte(e22);
	}

	return count;
}
