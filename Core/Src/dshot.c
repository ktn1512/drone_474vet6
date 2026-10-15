/**
 * @file dshot.c
 * @brief Đóng gói và truyền bốn khung DShot300 bằng các kênh PWM/DMA TIM5.
 */
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

/**
 * @brief Handle TIM5 do mã khởi tạo ngoại vi cung cấp, dùng chung cho bốn động cơ.
 */
extern TIM_HandleTypeDef htim5;

/**
 * @brief Số tick của một chu kỳ bit DShot300; giá trị giả định cho cấu hình TIM5.
 */
#define DSHOT300_TIMER_TICKS     480U
/**
 * @brief Số tick mức cao khi phát bit 0 trong chu kỳ 480 tick.
 */
#define DSHOT_BIT_0              180U
/**
 * @brief Số tick mức cao khi phát bit 1 trong chu kỳ 480 tick.
 */
#define DSHOT_BIT_1              360U
/**
 * @brief Số bit dữ liệu của một khung DShot.
 */
#define DSHOT_FRAME_WORDS        16U
/**
 * @brief Số mẫu compare gửi DMA: 16 bit khung và 2 chu kỳ mức thấp.
 */
#define DSHOT_DMA_WORDS          18U
/**
 * @brief Số động cơ/kênh PWM theo thiết kế; macro hiện chưa được dùng trong các vòng lặp.
 */
#define DSHOT_MOTOR_COUNT        4U

/**
 * @brief Bộ đệm compare TIM5 kênh 1, đơn vị tick bộ đếm.
 */
static uint16_t dma_ch1[DSHOT_DMA_WORDS];
/**
 * @brief Bộ đệm compare TIM5 kênh 2, đơn vị tick bộ đếm.
 */
static uint16_t dma_ch2[DSHOT_DMA_WORDS];
/**
 * @brief Bộ đệm compare TIM5 kênh 3, đơn vị tick bộ đếm.
 */
static uint16_t dma_ch3[DSHOT_DMA_WORDS];
/**
 * @brief Bộ đệm compare TIM5 kênh 4, đơn vị tick bộ đếm.
 */
static uint16_t dma_ch4[DSHOT_DMA_WORDS];

/**
 * @brief Cờ khung đang truyền DMA; được cập nhật cả trong callback hoàn tất.
 */
static volatile uint8_t dshot_dma_busy = 0U;
/**
 * @brief Mặt nạ kênh đã hoàn tất: bit 0–3 ứng với kênh 1–4; 0x0F là đủ bốn kênh.
 */
static volatile uint8_t dshot_dma_done_mask = 0U;

/**
 * @brief Đóng gói giá trị DShot, bit telemetry bằng 0 và checksum XOR 4 bit.
 * @param[in] throttle Giá trị DShot; giá trị lớn hơn 2047 được chặn về 2047.
 * @return Khung 16 bit gồm 11 bit giá trị, 1 bit telemetry và 4 bit checksum.
 * @note Hàm giữ nguyên các giá trị 0–47 nếu được truyền trực tiếp vào helper.
 */
static uint16_t DSHOT_MakePacket(uint16_t throttle)
{
    /**
     * @brief Phần giá trị và bit telemetry trước khi ghép checksum.
     */
    uint16_t packet;
    /**
     * @brief Bộ tích lũy XOR; sau cùng chỉ giữ 4 bit checksum.
     */
    uint16_t crc = 0U;
    /**
     * @brief Bản sao phần dữ liệu được dịch từng nibble để tính checksum.
     */
    uint16_t csum_data;

    if (throttle > 2047U)
        throttle = 2047U;

    packet = (uint16_t)(throttle << 1);
    csum_data = packet;

    /**
     * @brief Chỉ số nibble tham gia phép XOR checksum, từ 0 đến 2.
     */
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        crc ^= csum_data;
        csum_data >>= 4;
    }

    crc &= 0x0FU;
    return (uint16_t)((packet << 4) | crc);
}

/**
 * @brief Chuyển khung DShot thành mẫu compare PWM theo thứ tự bit cao trước.
 * @param[in] packet Khung DShot 16 bit đã chứa checksum.
 * @param[out] buffer Bộ đệm hợp lệ có ít nhất DSHOT_DMA_WORDS phần tử uint16_t.
 * @note Ghi 16 mẫu bit và hai mẫu duty bằng 0 tạo khoảng thấp cuối khung.
 */
static void DSHOT_FillBuffer(uint16_t packet, uint16_t *buffer)
{
    /**
     * @brief Chỉ số mẫu bit trong khung, từ 0 đến 15, tương ứng bit 15 đến 0.
     */
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

/**
 * @brief Ánh xạ lệnh PWM giả lập sang miền giá trị throttle DShot.
 * @param[in] pwm Độ rộng xung tương đương, µs; được chặn trong 1000–2000 µs.
 * @return Giá trị nguyên DShot từ 48 đến 2047 theo phép chia nguyên.
 * @note Không sinh mã dừng 0 hoặc các lệnh đặc biệt 1–47.
 */
static uint16_t PWM_To_DShot(uint16_t pwm)
{
    if (pwm < 1000U)
        pwm = 1000U;
    if (pwm > 2000U)
        pwm = 2000U;

    return (uint16_t)(48U + (((uint32_t)(pwm - 1000U) * 1999U) / 1000U));
}

/**
 * @brief Reset trạng thái truyền rồi khởi động PWM trên bốn kênh TIM5.
 */
void DSHOT_Init(void)
{
    dshot_dma_busy = 0U;
    dshot_dma_done_mask = 0U;

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
}

/**
 * @brief Chuẩn bị và gửi bốn khung bằng DMA nếu khung trước đã hoàn tất.
 */
void DSHOT_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    /**
     * @brief Khung DShot 16 bit của M1, gồm dữ liệu và checksum.
     */
    uint16_t p1;
    /**
     * @brief Khung DShot 16 bit của M2, gồm dữ liệu và checksum.
     */
    uint16_t p2;
    /**
     * @brief Khung DShot 16 bit của M3, gồm dữ liệu và checksum.
     */
    uint16_t p3;
    /**
     * @brief Khung DShot 16 bit của M4, gồm dữ liệu và checksum.
     */
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

/**
 * @brief Dừng DMA kênh TIM5 đã xong và mở khóa khi đủ bốn kênh hoàn tất.
 * @param[in,out] htim Handle timer do HAL truyền vào; phải là con trỏ hợp lệ.
 * @note Bỏ qua timer khác TIM5. Trường Channel chọn bit trong mặt nạ hoàn tất;
 *       callback được gọi trong luồng xử lý ngắt của HAL.
 */
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
