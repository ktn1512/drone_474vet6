/**
 * @file mmc5983ma.c
 * @brief Thực hiện đo từ trường từng lần và đổi mẫu MMC5983MA sang Gauss.
 */

/*
 * mmc5983ma.c
 * MMC5983MA magnetometer driver
 */

#include "mmc5983ma.h"

/**
 * @brief Địa chỉ byte cao của dữ liệu từ trường X.
 */
#define REG_XOUT0       0x00U
/**
 * @brief Địa chỉ byte kế tiếp của dữ liệu từ trường X.
 */
#define REG_XOUT1       0x01U
/**
 * @brief Địa chỉ byte cao của dữ liệu từ trường Y.
 */
#define REG_YOUT0       0x02U
/**
 * @brief Địa chỉ byte kế tiếp của dữ liệu từ trường Y.
 */
#define REG_YOUT1       0x03U
/**
 * @brief Địa chỉ byte cao của dữ liệu từ trường Z.
 */
#define REG_ZOUT0       0x04U
/**
 * @brief Địa chỉ byte kế tiếp của dữ liệu từ trường Z.
 */
#define REG_ZOUT1       0x05U
/**
 * @brief Địa chỉ byte chứa hai bit thấp của mỗi trục X/Y/Z.
 */
#define REG_XYZOUT2     0x06U
/**
 * @brief Địa chỉ thanh ghi trạng thái phép đo.
 */
#define REG_STATUS      0x08U
/**
 * @brief Địa chỉ thanh ghi lệnh đo và SET/RESET.
 */
#define REG_CONTROL0    0x09U
/**
 * @brief Địa chỉ thanh ghi điều khiển nội bộ 1.
 */
#define REG_CONTROL1    0x0AU
/**
 * @brief Địa chỉ thanh ghi điều khiển nội bộ 2.
 */
#define REG_CONTROL2    0x0BU
/**
 * @brief Địa chỉ thanh ghi Product ID.
 */
#define REG_PRODUCTID   0x2FU

/**
 * @brief Mặt nạ cờ hoàn tất phép đo từ trường trong thanh ghi STATUS.
 */
#define STATUS_MEAS_M_DONE  0x01U
/**
 * @brief Mặt nạ lệnh bắt đầu phép đo từ trường trong CONTROL0.
 */
#define CTRL0_TM_M          0x01U
/**
 * @brief Mặt nạ lệnh SET trong CONTROL0.
 */
#define CTRL0_SET           0x08U
/**
 * @brief Mặt nạ lệnh RESET trong CONTROL0.
 */
#define CTRL0_RESET         0x10U

/**
 * @brief Thời gian chờ mỗi giao dịch I2C của HAL, đơn vị ms.
 */
#define MMC5983_I2C_TIMEOUT_MS  100U
/**
 * @brief Mốc thời gian chờ cờ đo hoàn tất, đơn vị ms; vòng lặp kiểm tra sau mỗi lần poll.
 */
#define MMC5983_MEAS_TIMEOUT_MS 10U
/**
 * @brief Khoảng HAL_Delay giữa các lần kiểm tra trạng thái đo, đơn vị ms.
 */
#define MMC5983_POLL_DELAY_MS   1U

/**
 * @brief Ghi một byte thanh ghi qua I2C HAL.
 * @param[in] dev Trạng thái có handle I2C hợp lệ.
 * @param[in] reg Địa chỉ thanh ghi một byte.
 * @param[in] data Byte cần ghi.
 * @return HAL_ERROR nếu dev/hi2c không hợp lệ; nếu hợp lệ trả nguyên trạng thái HAL_I2C_Mem_Write.
 */
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

/**
 * @brief Đọc liên tiếp các byte thanh ghi qua I2C HAL.
 * @param[in] dev Trạng thái có handle I2C hợp lệ.
 * @param[in] reg Địa chỉ thanh ghi bắt đầu.
 * @param[out] data Bộ đệm nhận ít nhất len byte.
 * @param[in] len Số byte cần nhận, phải lớn hơn 0.
 * @return HAL_ERROR nếu tham số không hợp lệ; nếu hợp lệ trả nguyên trạng thái HAL_I2C_Mem_Read.
 */
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

/**
 * @brief Đọc Product ID và trả 0 khi giao dịch thất bại.
 * @see MMC5983_WhoAmI trong mmc5983ma.h để biết tham số và mã trả về.
 */
uint8_t MMC5983_WhoAmI(MMC5983MA_t *dev)
{
    /**
     * @brief Product ID đọc từ I2C; giá trị mặc định 0 trước khi đọc.
     */
    uint8_t id = 0U;

    if (ReadReg(dev, REG_PRODUCTID, &id, 1U) != HAL_OK)
        return 0U;

    return id;
}

/**
 * @brief Kiểm tra ID, thực hiện RESET/SET và ghi cấu hình đo.
 * @see MMC5983_Init trong mmc5983ma.h để biết tham số và mã trả về.
 */
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

    /**
     * @brief Product ID dùng kiểm tra cảm biến trong quá trình khởi tạo.
     */
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

/**
 * @brief Đợi một phép đo và cập nhật ba giá trị thô 18 bit.
 * @see MMC5983_ReadRaw trong mmc5983ma.h để biết tham số và mã trả về.
 */
HAL_StatusTypeDef MMC5983_ReadRaw(MMC5983MA_t *dev,
                                  int32_t *x,
                                  int32_t *y,
                                  int32_t *z)
{
    if ((dev == NULL) || (dev->hi2c == NULL) ||
        (x == NULL) || (y == NULL) || (z == NULL))
        return HAL_ERROR;

    /**
     * @brief Byte trạng thái dùng kiểm tra bit hoàn tất phép đo từ trường.
     */
    uint8_t status = 0U;
    /**
     * @brief Bảy byte gồm sáu byte trục và một byte chứa các bit thấp.
     */
    uint8_t buf[7] = {0U};

    if (WriteReg(dev, REG_CONTROL0, CTRL0_TM_M) != HAL_OK)
        return HAL_ERROR;

    /**
     * @brief Tick bắt đầu chờ phép đo, đơn vị ms của HAL_GetTick.
     */
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

    /**
     * @brief Mẫu X ghép đủ 18 bit, đơn vị LSB không dấu.
     */
    uint32_t raw_x = ((uint32_t)buf[0] << 10) |
                     ((uint32_t)buf[1] << 2) |
                     ((buf[6] >> 6) & 0x03U);

    /**
     * @brief Mẫu Y ghép đủ 18 bit, đơn vị LSB không dấu.
     */
    uint32_t raw_y = ((uint32_t)buf[2] << 10) |
                     ((uint32_t)buf[3] << 2) |
                     ((buf[6] >> 4) & 0x03U);

    /**
     * @brief Mẫu Z ghép đủ 18 bit, đơn vị LSB không dấu.
     */
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

/**
 * @brief Đọc raw rồi trừ điểm giữa và chia hệ số để xuất Gauss.
 * @see MMC5983_ReadMag trong mmc5983ma.h để biết tham số và mã trả về.
 */
HAL_StatusTypeDef MMC5983_ReadMag(MMC5983MA_t *dev,
                                  float *x,
                                  float *y,
                                  float *z)
{
    if ((dev == NULL) || (x == NULL) || (y == NULL) || (z == NULL))
        return HAL_ERROR;

    /**
     * @brief Dữ liệu X thô 18 bit nhận từ ReadRaw, đơn vị LSB.
     */
    int32_t raw_x;
    /**
     * @brief Dữ liệu Y thô 18 bit nhận từ ReadRaw, đơn vị LSB.
     */
    int32_t raw_y;
    /**
     * @brief Dữ liệu Z thô 18 bit nhận từ ReadRaw, đơn vị LSB.
     */
    int32_t raw_z;

    /**
     * @brief Kết quả ReadRaw; chỉ đổi đơn vị nếu phép đo thành công.
     */
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