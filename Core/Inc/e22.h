/*
 * e22.h
 *
 *  Created on: 23 thg 8, 2026
 *      Author: khanh
 */

#ifndef __E22_H
#define __E22_H

#include "main.h"
#include <stdint.h>

#define E22_RX_BUFFER_SIZE 256

typedef struct {
	UART_HandleTypeDef *huart;

	uint8_t rx_byte;

	uint8_t rx_buffer[E22_RX_BUFFER_SIZE];
	volatile uint16_t rx_head;
	volatile uint16_t rx_tail;

} E22_t;

void E22_Init(E22_t *e22, UART_HandleTypeDef *huart);

HAL_StatusTypeDef E22_Send(E22_t *e22, const uint8_t *data, uint16_t len);

HAL_StatusTypeDef E22_SendString(E22_t *e22, const char *str);

void E22_RxCallback(E22_t *e22);

uint16_t E22_Available(E22_t *e22);

uint8_t E22_ReadByte(E22_t *e22);

uint16_t E22_Read(E22_t *e22, uint8_t *buf, uint16_t max_len);

#endif /* INC_E22_H_ */
