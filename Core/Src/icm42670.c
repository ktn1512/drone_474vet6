/*
 * icm42670.c
 * ICM-42670-P IMU driver
 */

#include "icm42670.h"
#include <string.h>

static void CS_Low(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

static void CS_High(void)
{
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}

ICM42670_Status_t ICM42670_ReadReg(uint8_t reg, uint8_t *data)
{
    if (data == NULL)
        return ICM42670_ERROR;

    uint8_t addr = reg | 0x80U;
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

ICM42670_Status_t ICM42670_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { (uint8_t)(reg & 0x7FU), data };
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

static ICM42670_Status_t ICM42670_ReadBytes(uint8_t reg, uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
        return ICM42670_ERROR;

    uint8_t addr = reg | 0x80U;
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

static int16_t MakeInt16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

ICM42670_Status_t ICM42670_Init(ICM42670_t *dev)
{
    if (dev == NULL)
        return ICM42670_ERROR;

    memset(dev, 0, sizeof(*dev));
    CS_High();
    HAL_Delay(10U);

    uint8_t who_am_i = 0U;
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

ICM42670_Status_t ICM42670_ReadRaw(ICM42670_Data_t *imu)
{
    if (imu == NULL)
        return ICM42670_ERROR;

    uint8_t buf[14];
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

ICM42670_Status_t ICM42670_ReadAll(ICM42670_Data_t *imu)
{
    if (imu == NULL)
        return ICM42670_ERROR;

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

void ICM42670_ApplyCalibration(ICM42670_t *dev, ICM42670_Data_t *imu)
{
    if ((dev == NULL) || (imu == NULL))
        return;

    imu->gx_dps -= dev->gyro_bias_x_dps;
    imu->gy_dps -= dev->gyro_bias_y_dps;
    imu->gz_dps -= dev->gyro_bias_z_dps;
}

ICM42670_Status_t ICM42670_CalibrateGyro(ICM42670_t *dev, uint16_t samples, uint16_t sample_delay_ms)
{
    if ((dev == NULL) || (samples == 0U))
        return ICM42670_ERROR;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    ICM42670_Data_t imu;

    for (uint16_t i = 0U; i < samples; ++i)
    {
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
