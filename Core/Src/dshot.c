/*
 * dshot.c
 * DShot300 output driver using TIM5 PWM + DMA
 *
 * Timer assumption:
 *   TIM5 counter clock = 144 MHz
 *   Prescaler = 0
 *   ARR = 479
 *
 * Therefore:
 *   Timer tick = 6.944 ns
 *   DShot300 bit period = 3.333 us = 480 ticks
 *   Bit 0 high time ~= 37.5% = 180 ticks
 *   Bit 1 high time ~= 75.0%  = 360 ticks
 */

#include "dshot.h"

extern TIM_HandleTypeDef htim5;

#define DSHOT300_TIMER_TICKS     480U
#define DSHOT_BIT_0              180U
#define DSHOT_BIT_1              360U
#define DSHOT_FRAME_WORDS        16U
#define DSHOT_DMA_WORDS          18U
#define DSHOT_MOTOR_COUNT        4U

static uint16_t dma_ch1[DSHOT_DMA_WORDS];
static uint16_t dma_ch2[DSHOT_DMA_WORDS];
static uint16_t dma_ch3[DSHOT_DMA_WORDS];
static uint16_t dma_ch4[DSHOT_DMA_WORDS];

static volatile uint8_t dshot_dma_busy = 0U;
static volatile uint8_t dshot_dma_done_mask = 0U;

static uint16_t DSHOT_MakePacket(uint16_t throttle)
{
    uint16_t packet;
    uint16_t crc = 0U;
    uint16_t csum_data;

    if (throttle > 2047U)
        throttle = 2047U;

    packet = (uint16_t)(throttle << 1);
    csum_data = packet;

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        crc ^= csum_data;
        csum_data >>= 4;
    }

    crc &= 0x0FU;
    return (uint16_t)((packet << 4) | crc);
}

static void DSHOT_FillBuffer(uint16_t packet, uint16_t *buffer)
{
    for (uint8_t i = 0U; i < DSHOT_FRAME_WORDS; ++i)
    {
        buffer[i] = ((packet & (1U << (15U - i))) != 0U)
                  ? DSHOT_BIT_1
                  : DSHOT_BIT_0;
    }

    /* Two zero-duty periods provide the inter-frame low time. */
    buffer[16] = 0U;
    buffer[17] = 0U;
}

static uint16_t PWM_To_DShot(uint16_t pwm)
{
    if (pwm < 1000U)
        pwm = 1000U;
    if (pwm > 2000U)
        pwm = 2000U;

    return (uint16_t)(48U + (((uint32_t)(pwm - 1000U) * 1999U) / 1000U));
}

void DSHOT_Init(void)
{
    dshot_dma_busy = 0U;
    dshot_dma_done_mask = 0U;

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

void DSHOT_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    uint16_t p1;
    uint16_t p2;
    uint16_t p3;
    uint16_t p4;

    /* Do not overwrite a frame while the previous DMA transfers are active. */
    if (dshot_dma_busy != 0U)
        return;

    p1 = DSHOT_MakePacket(PWM_To_DShot(m1));
    p2 = DSHOT_MakePacket(PWM_To_DShot(m2));
    p3 = DSHOT_MakePacket(PWM_To_DShot(m3));
    p4 = DSHOT_MakePacket(PWM_To_DShot(m4));

    DSHOT_FillBuffer(p1, dma_ch1);
    DSHOT_FillBuffer(p2, dma_ch2);
    DSHOT_FillBuffer(p3, dma_ch3);
    DSHOT_FillBuffer(p4, dma_ch4);

    dshot_dma_done_mask = 0U;
    dshot_dma_busy = 1U;

    if (HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_1,
                              (uint32_t *)dma_ch1, DSHOT_DMA_WORDS) != HAL_OK)
        goto dma_error;

    if (HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_2,
                              (uint32_t *)dma_ch2, DSHOT_DMA_WORDS) != HAL_OK)
        goto dma_error;

    if (HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_3,
                              (uint32_t *)dma_ch3, DSHOT_DMA_WORDS) != HAL_OK)
        goto dma_error;

    if (HAL_TIM_PWM_Start_DMA(&htim5, TIM_CHANNEL_4,
                              (uint32_t *)dma_ch4, DSHOT_DMA_WORDS) != HAL_OK)
        goto dma_error;

    return;

dma_error:
    HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_4);
    dshot_dma_done_mask = 0U;
    dshot_dma_busy = 0U;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim5)
        return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_1);
        dshot_dma_done_mask |= 0x01U;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_2);
        dshot_dma_done_mask |= 0x02U;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_3);
        dshot_dma_done_mask |= 0x04U;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        HAL_TIM_PWM_Stop_DMA(&htim5, TIM_CHANNEL_4);
        dshot_dma_done_mask |= 0x08U;
    }

    if (dshot_dma_done_mask == 0x0FU)
        dshot_dma_busy = 0U;
}
