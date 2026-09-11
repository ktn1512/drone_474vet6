/*
 * dshot.c
 *
 *  Created on: 30 thg 8, 2026
 *      Author: khanh
 */

#include "dshot.h"

extern TIM_HandleTypeDef htim5;

#define DSHOT_BIT_0 180
#define DSHOT_BIT_1 360

static uint16_t dma_ch1[18];
static uint16_t dma_ch2[18];
static uint16_t dma_ch3[18];
static uint16_t dma_ch4[18];

static uint16_t DSHOT_MakePacket(uint16_t throttle) {
	uint16_t packet;
	uint16_t crc = 0;
	uint16_t csum_data;

	if (throttle > 2047)
		throttle = 2047;

	packet = throttle << 1;

	csum_data = packet;

	for (int i = 0; i < 3; i++) {
		crc ^= csum_data;
		csum_data >>= 4;
	}

	crc &= 0x0F;

	return (packet << 4) | crc;
}

static void DSHOT_FillBuffer(uint16_t packet, uint16_t *buffer) {
	for (int i = 0; i < 16; i++) {
		if (packet & (1 << (15 - i)))
			buffer[i] = DSHOT_BIT_1;
		else
			buffer[i] = DSHOT_BIT_0;
	}

	buffer[16] = 0;
	buffer[17] = 0;
}

static uint16_t PWM_To_DShot(uint16_t pwm) {
	if (pwm < 1000)
		pwm = 1000;
	if (pwm > 2000)
		pwm = 2000;

	return 48 + ((uint32_t) (pwm - 1000) * 1999) / 1000;
}

void DSHOT_Init(void) {
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

void DSHOT_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
	uint16_t p1 = DSHOT_MakePacket(PWM_To_DShot(m1));
	uint16_t p2 = DSHOT_MakePacket(PWM_To_DShot(m2));
	uint16_t p3 = DSHOT_MakePacket(PWM_To_DShot(m3));
	uint16_t p4 = DSHOT_MakePacket(PWM_To_DShot(m4));

	DSHOT_FillBuffer(p1, dma_ch1);
	DSHOT_FillBuffer(p2, dma_ch2);
	DSHOT_FillBuffer(p3, dma_ch3);
	DSHOT_FillBuffer(p4, dma_ch4);

	HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_1, (uint32_t*) dma_ch1, 18);

	HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2, (uint32_t*) dma_ch2, 18);

	HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3, (uint32_t*) dma_ch3, 18);

	HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_4, (uint32_t*) dma_ch4, 18);
}


