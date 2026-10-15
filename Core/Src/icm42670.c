/**
 * @file icm42670.c
 * @brief Thực hiện giao dịch SPI, đọc mẫu và tính gyro bias cho ICM-42670.
 */

/*
 * icm42670.c
 * ICM-42670-P IMU driver
 */

#include "icm42670.h"
#include <string.h>

/**
 * @brief Kéo chip-select xuống thấp để bắt đầu giao dịch SPI.
 */
static void CS_Low(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Đưa chip-select lên cao để kết thúc giao dịch SPI.
 */
static void CS_High(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Đọc một thanh ghi và chuyển kết quả HAL sang mã trạng thái driver.
 * @see ICM42670_ReadReg trong icm42670.h để biết tham số và kết quả.
 */
ICM42670_Status_t ICM42670_ReadReg(uint8_t reg, uint8_t *data)
{
    if (data == NULL)
        return ICM42670_ERROR;

    /**
     * @brief Byte địa chỉ SPI có bit 7 đặt để chọn thao tác đọc.
     */
    uint8_t addr = reg | 0x80U;
    /**
     * @brief Kết quả giao dịch SPI gần nhất để xử lý lỗi hoặc timeout.
     */
    HAL_StatusTypeDef status;

    CS_Low();
    status = HAL_SPI_Transmit(&ICM42670_SPI, &addr, 1U, 100U);
    if (status == HAL_OK)
        status = HAL_SPI_Receive(&ICM42670_SPI, data, 1U, 100U);
    CS_High();

    if (status == HAL_TIMEOUT)
        return ICM42670_TIMEOUT;
    if (status != HAL_OK)
        return ICM42670_ERROR;

    return ICM42670_OK;
}

/**
 * @brief Ghi một thanh ghi bằng giao dịch SPI hai byte.
 * @see ICM42670_WriteReg trong icm42670.h để biết tham số và kết quả.
 */
ICM42670_Status_t ICM42670_WriteReg(uint8_t reg, uint8_t data)
{
    /**
     * @brief Hai byte gửi SPI gồm địa chỉ ghi và giá trị thanh ghi.
     */
    uint8_t buf[2] = { (uint8_t)(reg & 0x7FU), data };
    /**
     * @brief Kết quả giao dịch SPI gần nhất để xử lý lỗi hoặc timeout.
     */
    HAL_StatusTypeDef status;

    CS_Low();
    status = HAL_SPI_Transmit(&ICM42670_SPI, buf, 2U, 100U);
    CS_High();

    if (status == HAL_TIMEOUT)
        return ICM42670_TIMEOUT;
    if (status != HAL_OK)
        return ICM42670_ERROR;

    return ICM42670_OK;
}

/**
 * @brief Đọc liên tiếp nhiều byte kể từ một thanh ghi qua SPI.
 * @param[in] reg Địa chỉ thanh ghi bắt đầu, bit đọc được tự thêm.
 * @param[out] data Bộ đệm chứa ít nhất len byte; không được NULL.
 * @param[in] len Số byte cần đọc; phải lớn hơn 0.
 * @retval ICM42670_OK Đã nhận đủ byte theo kết quả HAL.
 * @retval ICM42670_TIMEOUT Một giao dịch SPI trả về HAL_TIMEOUT.
 * @retval ICM42670_ERROR Tham số không hợp lệ hoặc lỗi SPI khác.
 */
static ICM42670_Status_t ICM42670_ReadBytes(uint8_t reg, uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
        return ICM42670_ERROR;

    /**
     * @brief Byte địa chỉ SPI có bit 7 đặt để chọn thao tác đọc.
     */
    uint8_t addr = reg | 0x80U;
    /**
     * @brief Kết quả giao dịch SPI gần nhất để xử lý lỗi hoặc timeout.
     */
    HAL_StatusTypeDef status;

    CS_Low();
    status = HAL_SPI_Transmit(&ICM42670_SPI, &addr, 1U, 100U);
    if (status == HAL_OK)
        status = HAL_SPI_Receive(&ICM42670_SPI, data, len, 100U);
    CS_High();

    if (status == HAL_TIMEOUT)
        return ICM42670_TIMEOUT;
    if (status != HAL_OK)
        return ICM42670_ERROR;

    return ICM42670_OK;
}

/**
 * @brief Ghép hai byte cao/thấp thành một giá trị 16 bit có dấu.
 * @param[in] msb Byte cao của mẫu.
 * @param[in] lsb Byte thấp của mẫu.
 * @return Giá trị int16_t mang mẫu bit của (msb << 8) | lsb.
 */
static int16_t MakeInt16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

/**
 * @brief Kiểm tra cảm biến và ghi cấu hình gyro/accel mặc định.
 * @see ICM42670_Init trong icm42670.h để biết tham số và kết quả.
 */
ICM42670_Status_t ICM42670_Init(ICM42670_t *dev)
{
    if (dev == NULL)
        return ICM42670_ERROR;

    memset(dev, 0, sizeof(*dev));
    CS_High();
    HAL_Delay(10U);

    /**
     * @brief Byte nhận dạng thiết bị để so với ICM_WHO_AM_I_VALUE.
     */
    uint8_t who_am_i = 0U;
    /**
     * @brief Kết quả của từng thao tác đọc ID/ghi cấu hình khởi tạo.
     */
    ICM42670_Status_t status = ICM42670_ReadReg(ICM_WHO_AM_I, &who_am_i);
    if (status != ICM42670_OK)
        return status;

    if (who_am_i != ICM_WHO_AM_I_VALUE)
        return ICM42670_WHOAMI_ERROR;

    /* Gyro: +/-2000 dps, 1.6 kHz */
    status = ICM42670_WriteReg(ICM_GYRO_CONFIG0, 0x05U);
    if (status != ICM42670_OK)
        return status;

    /* Accel: +/-8 g, 1.6 kHz */
    status = ICM42670_WriteReg(ICM_ACCEL_CONFIG0, 0x25U);
    if (status != ICM42670_OK)
        return status;

    /* Gyro UI LPF: 53 Hz. Reserved bits remain 0. */
    status = ICM42670_WriteReg(ICM_GYRO_CONFIG1, 0x04U);
    if (status != ICM42670_OK)
        return status;

    /* Accel UI LPF: 53 Hz, 2x averaging. Reserved bits remain 0. */
    status = ICM42670_WriteReg(ICM_ACCEL_CONFIG1, 0x04U);
    if (status != ICM42670_OK)
        return status;

    /* Gyro + accel in low-noise mode */
    status = ICM42670_WriteReg(ICM_PWR_MGMT0, 0x0FU);
    if (status != ICM42670_OK)
        return status;

    HAL_Delay(50U);
    dev->initialized = 1U;

    return ICM42670_OK;
}

/**
 * @brief Đọc và tách mẫu nhiệt độ/accel/gyro thô từ khối thanh ghi.
 * @see ICM42670_ReadRaw trong icm42670.h để biết tham số và kết quả.
 */
ICM42670_Status_t ICM42670_ReadRaw(ICM42670_Data_t *imu)
{
    if (imu == NULL)
        return ICM42670_ERROR;

    /**
     * @brief Khối 14 byte: nhiệt độ, accel XYZ và gyro XYZ, mỗi mẫu hai byte.
     */
    uint8_t buf[14];
    /**
     * @brief Kết quả đọc khối raw; chỉ giải mã khi giao dịch thành công.
     */
    ICM42670_Status_t status = ICM42670_ReadBytes(ICM_TEMP_DATA1, buf, sizeof(buf));
    if (status != ICM42670_OK)
        return status;

    /* TEMP, ACCEL XYZ, GYRO XYZ are contiguous from 0x09 to 0x16. */
    imu->temp_raw = MakeInt16(buf[0], buf[1]);
    imu->ax_raw = MakeInt16(buf[2], buf[3]);
    imu->ay_raw = MakeInt16(buf[4], buf[5]);
    imu->az_raw = MakeInt16(buf[6], buf[7]);
    imu->gx_raw = MakeInt16(buf[8], buf[9]);
    imu->gy_raw = MakeInt16(buf[10], buf[11]);
    imu->gz_raw = MakeInt16(buf[12], buf[13]);

    return ICM42670_OK;
}

/**
 * @brief Đọc raw và đổi đơn vị; gyro bias chưa được áp dụng ở bước này.
 * @see ICM42670_ReadAll trong icm42670.h để biết tham số và kết quả.
 */
ICM42670_Status_t ICM42670_ReadAll(ICM42670_Data_t *imu)
{
    if (imu == NULL)
        return ICM42670_ERROR;

    /**
     * @brief Kết quả đọc mẫu raw trước khi thực hiện đổi đơn vị.
     */
    ICM42670_Status_t status = ICM42670_ReadRaw(imu);
    if (status != ICM42670_OK)
        return status;

    imu->ax_g = (float)imu->ax_raw / ICM_ACCEL_LSB_PER_G;
    imu->ay_g = (float)imu->ay_raw / ICM_ACCEL_LSB_PER_G;
    imu->az_g = (float)imu->az_raw / ICM_ACCEL_LSB_PER_G;

    imu->gx_dps = (float)imu->gx_raw / ICM_GYRO_LSB_PER_DPS;
    imu->gy_dps = (float)imu->gy_raw / ICM_GYRO_LSB_PER_DPS;
    imu->gz_dps = (float)imu->gz_raw / ICM_GYRO_LSB_PER_DPS;

    imu->temperature_c = ((float)imu->temp_raw / 132.48f) + 25.0f;

    return ICM42670_OK;
}

/**
 * @brief Trừ ba gyro bias khỏi dữ liệu vận tốc góc đã đổi đơn vị.
 * @see ICM42670_ApplyCalibration trong icm42670.h để biết tham số và kết quả.
 */
void ICM42670_ApplyCalibration(ICM42670_t *dev, ICM42670_Data_t *imu)
{
    if ((dev == NULL) || (imu == NULL))
        return;

    imu->gx_dps -= dev->gyro_bias_x_dps;
    imu->gy_dps -= dev->gyro_bias_y_dps;
    imu->gz_dps -= dev->gyro_bias_z_dps;
}

/**
 * @brief Lấy trung bình gyro chưa trừ bias để ghi các độ lệch tĩnh.
 * @see ICM42670_CalibrateGyro trong icm42670.h để biết tham số và kết quả.
 */
ICM42670_Status_t ICM42670_CalibrateGyro(ICM42670_t *dev, uint16_t samples, uint16_t sample_delay_ms)
{
    if ((dev == NULL) || (samples == 0U))
        return ICM42670_ERROR;

    /**
     * @brief Tổng các mẫu vận tốc góc X, đơn vị độ/giây trước bước chia lấy trung bình.
     */
    float sum_x = 0.0f;
    /**
     * @brief Tổng các mẫu vận tốc góc Y, đơn vị độ/giây trước bước chia lấy trung bình.
     */
    float sum_y = 0.0f;
    /**
     * @brief Tổng các mẫu vận tốc góc Z, đơn vị độ/giây trước bước chia lấy trung bình.
     */
    float sum_z = 0.0f;
    /**
     * @brief Mẫu IMU tạm với gyro chưa trừ bias để tính độ lệch tĩnh.
     */
    ICM42670_Data_t imu;

    /**
     * @brief Chỉ số mẫu đã lấy, từ 0 tới samples - 1.
     */
    for (uint16_t i = 0U; i < samples; ++i)
    {
        /**
         * @brief Kết quả đọc mẫu hiện tại; lỗi làm dừng hiệu chuẩn trước khi ghi bias.
         */
        ICM42670_Status_t status = ICM42670_ReadAll(&imu);
        if (status != ICM42670_OK)
            return status;

        sum_x += imu.gx_dps;
        sum_y += imu.gy_dps;
        sum_z += imu.gz_dps;

        if (sample_delay_ms > 0U)
            HAL_Delay(sample_delay_ms);
    }

    dev->gyro_bias_x_dps = sum_x / (float)samples;
    dev->gyro_bias_y_dps = sum_y / (float)samples;
    dev->gyro_bias_z_dps = sum_z / (float)samples;

    return ICM42670_OK;
}
