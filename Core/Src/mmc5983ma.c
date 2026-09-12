/*
 * mmc5983ma.c
 * MMC5983MA magnetometer driver
 */

#include "mmc5983ma.h"

#define REG_XOUT0       0x00U
#define REG_XOUT1       0x01U
#define REG_YOUT0       0x02U
#define REG_YOUT1       0x03U
#define REG_ZOUT0       0x04U
#define REG_ZOUT1       0x05U
#define REG_XYZOUT2     0x06U
#define REG_STATUS      0x08U
#define REG_CONTROL0    0x09U
#define REG_CONTROL1    0x0AU
#define REG_CONTROL2    0x0BU
#define REG_PRODUCTID   0x2FU

#define STATUS_MEAS_M_DONE  0x01U
#define CTRL0_TM_M          0x01U
#define CTRL0_SET           0x08U
#define CTRL0_RESET         0x10U

#define MMC5983_I2C_TIMEOUT_MS  100U
#define MMC5983_MEAS_TIMEOUT_MS 10U
#define MMC5983_POLL_DELAY_MS   1U

static HAL_StatusTypeDef WriteReg(MMC5983MA_t *dev, uint8_t reg, uint8_t data)
{
    if ((dev == NULL) || (dev->hi2c == NULL))
        return HAL_ERROR;

    return HAL_I2C_Mem_Write(dev->hi2c,
                             MMC5983_I2C_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             1U,
                             MMC5983_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef ReadReg(MMC5983MA_t *dev,
                                 uint8_t reg,
                                 uint8_t *data,
                                 uint16_t len)
{
    if ((dev == NULL) || (dev->hi2c == NULL) || (data == NULL) || (len == 0U))
        return HAL_ERROR;

    return HAL_I2C_Mem_Read(dev->hi2c,
                            MMC5983_I2C_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            MMC5983_I2C_TIMEOUT_MS);
}

uint8_t MMC5983_WhoAmI(MMC5983MA_t *dev)
{
    uint8_t id = 0U;

    if (ReadReg(dev, REG_PRODUCTID, &id, 1U) != HAL_OK)
        return 0U;

    return id;
}

HAL_StatusTypeDef MMC5983_Init(MMC5983MA_t *dev)
{
    if ((dev == NULL) || (dev->hi2c == NULL))
        return HAL_ERROR;

    dev->raw_x = 0;
    dev->raw_y = 0;
    dev->raw_z = 0;
    dev->mag_x = 0.0f;
    dev->mag_y = 0.0f;
    dev->mag_z = 0.0f;

    uint8_t id = MMC5983_WhoAmI(dev);
    if (id != MMC5983_WHOAMI)
        return HAL_ERROR;

    HAL_Delay(1U);

    if (WriteReg(dev, REG_CONTROL0, CTRL0_RESET) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(1U);

    if (WriteReg(dev, REG_CONTROL0, CTRL0_SET) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(1U);

    if (WriteReg(dev, REG_CONTROL1, 0x00U) != HAL_OK)
        return HAL_ERROR;

    if (WriteReg(dev, REG_CONTROL2, 0x00U) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef MMC5983_ReadRaw(MMC5983MA_t *dev,
                                  int32_t *x,
                                  int32_t *y,
                                  int32_t *z)
{
    if ((dev == NULL) || (dev->hi2c == NULL) ||
        (x == NULL) || (y == NULL) || (z == NULL))
        return HAL_ERROR;

    uint8_t status = 0U;
    uint8_t buf[7] = {0U};

    if (WriteReg(dev, REG_CONTROL0, CTRL0_TM_M) != HAL_OK)
        return HAL_ERROR;

    uint32_t start = HAL_GetTick();

    do
    {
        if (ReadReg(dev, REG_STATUS, &status, 1U) != HAL_OK)
            return HAL_ERROR;

        if ((status & STATUS_MEAS_M_DONE) != 0U)
            break;

        HAL_Delay(MMC5983_POLL_DELAY_MS);

    } while ((HAL_GetTick() - start) <= MMC5983_MEAS_TIMEOUT_MS);

    if ((status & STATUS_MEAS_M_DONE) == 0U)
        return HAL_TIMEOUT;

    if (ReadReg(dev, REG_XOUT0, buf, sizeof(buf)) != HAL_OK)
        return HAL_ERROR;

    uint32_t raw_x = ((uint32_t)buf[0] << 10) |
                     ((uint32_t)buf[1] << 2) |
                     ((buf[6] >> 6) & 0x03U);

    uint32_t raw_y = ((uint32_t)buf[2] << 10) |
                     ((uint32_t)buf[3] << 2) |
                     ((buf[6] >> 4) & 0x03U);

    uint32_t raw_z = ((uint32_t)buf[4] << 10) |
                     ((uint32_t)buf[5] << 2) |
                     ((buf[6] >> 2) & 0x03U);

    *x = (int32_t)raw_x;
    *y = (int32_t)raw_y;
    *z = (int32_t)raw_z;

    dev->raw_x = *x;
    dev->raw_y = *y;
    dev->raw_z = *z;

    return HAL_OK;
}

HAL_StatusTypeDef MMC5983_ReadMag(MMC5983MA_t *dev,
                                  float *x,
                                  float *y,
                                  float *z)
{
    if ((dev == NULL) || (x == NULL) || (y == NULL) || (z == NULL))
        return HAL_ERROR;

    int32_t raw_x;
    int32_t raw_y;
    int32_t raw_z;

    HAL_StatusTypeDef status = MMC5983_ReadRaw(dev, &raw_x, &raw_y, &raw_z);
    if (status != HAL_OK)
        return status;

    *x = ((float)raw_x - 131072.0f) / 16384.0f;
    *y = ((float)raw_y - 131072.0f) / 16384.0f;
    *z = ((float)raw_z - 131072.0f) / 16384.0f;

    dev->mag_x = *x;
    dev->mag_y = *y;
    dev->mag_z = *z;

    return HAL_OK;
}