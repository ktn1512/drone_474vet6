/*
 * mmc5983ma.c
 *
 *  Created on: 22 thg 8, 2026
 *      Author: khanh
 */

#include "mmc5983ma.h"

/* =========================
 * Thanh ghi MMC5983MA
 * ========================= */

#define REG_XOUT0 0x00 // X[17:10]
#define REG_XOUT1 0x01 // X[9:2]

#define REG_YOUT0 0x02 // Y[17:10]
#define REG_YOUT1 0x03 // Y[9:2]

#define REG_ZOUT0 0x04 // Z[17:10]
#define REG_ZOUT1 0x05 // Z[9:2]

#define REG_XYZOUT2 0x06 // 2 bit cuối của X,Y,Z

#define REG_STATUS 0x08 // Trạng thái đo

#define REG_CONTROL0 0x09 // Control 0
#define REG_CONTROL1 0x0A // Control 1
#define REG_CONTROL2 0x0B // Control 2

#define REG_PRODUCTID 0x2F // Product ID

/* Bit STATUS */
#define STATUS_MEAS_M_DONE 0x01

/* Bit CONTROL0 */
#define CTRL0_TM_M 0x01	 // Bắt đầu đo từ trường
#define CTRL0_SET 0x08	 // SET
#define CTRL0_RESET 0x10 // RESET

/* =========================================================
 * Ghi 1 byte vào thanh ghi
 *
 * dev  : cảm biến
 * reg  : địa chỉ thanh ghi
 * data : dữ liệu cần ghi
 * ========================================================= */
static HAL_StatusTypeDef WriteReg(
	MMC5983MA_t *dev,
	uint8_t reg,
	uint8_t data)
{
	return HAL_I2C_Mem_Write(
		dev->hi2c,
		MMC5983_I2C_ADDR,
		reg,
		I2C_MEMADD_SIZE_8BIT,
		&data,
		1,
		100);
}

/* =========================================================
 * Đọc nhiều byte từ thanh ghi
 *
 * dev  : cảm biến
 * reg  : thanh ghi bắt đầu đọc
 * data : buffer chứa dữ liệu đọc được
 * len  : số byte cần đọc
 * ========================================================= */
static HAL_StatusTypeDef ReadReg(
	MMC5983MA_t *dev,
	uint8_t reg,
	uint8_t *data,
	uint16_t len)
{
	return HAL_I2C_Mem_Read(
		dev->hi2c,
		MMC5983_I2C_ADDR,
		reg,
		I2C_MEMADD_SIZE_8BIT,
		data,
		len,
		100);
}

/* =========================================================
 * Đọc Product ID
 *
 * dev : cảm biến
 *
 * Trả về 0x30 nếu đúng MMC5983MA
 * ========================================================= */
uint8_t MMC5983_WhoAmI(MMC5983MA_t *dev)
{
	uint8_t id = 0;

	ReadReg(
		dev,
		REG_PRODUCTID,
		&id,
		1);

	return id;
}

/* =========================================================
 * Khởi tạo MMC5983MA
 *
 * dev : con trỏ tới cảm biến
 *
 * Kiểm tra ID và cấu hình cơ bản.
 * ========================================================= */
HAL_StatusTypeDef MMC5983_Init(MMC5983MA_t *dev)
{
	/* Kiểm tra Product ID */
	if (MMC5983_WhoAmI(dev) != MMC5983_WHOAMI)
	{
		return HAL_ERROR;
	}

	HAL_Delay(10);

	/* RESET từ kế */
	if (WriteReg(
			dev,
			REG_CONTROL0,
			CTRL0_RESET) != HAL_OK)
	{
		return HAL_ERROR;
	}

	HAL_Delay(1);

	/* SET từ kế */
	if (WriteReg(
			dev,
			REG_CONTROL0,
			CTRL0_SET) != HAL_OK)
	{
		return HAL_ERROR;
	}

	HAL_Delay(1);

	/* Control1 = 0
	 * Không dùng continuous mode */
	if (WriteReg(
			dev,
			REG_CONTROL1,
			0x00) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* Control2 = 0
	 * Đọc từng lần bằng TM_M */
	if (WriteReg(
			dev,
			REG_CONTROL2,
			0x00) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

/* =========================================================
 * Đọc dữ liệu raw X/Y/Z
 *
 * dev : cảm biến
 * x   : lưu raw X
 * y   : lưu raw Y
 * z   : lưu raw Z
 *
 * Trả về:
 * HAL_OK      : đọc thành công
 * HAL_ERROR   : lỗi I2C
 * HAL_TIMEOUT : cảm biến không hoàn thành phép đo
 * ========================================================= */
HAL_StatusTypeDef MMC5983_ReadRaw(
	MMC5983MA_t *dev,
	int32_t *x,
	int32_t *y,
	int32_t *z)
{
	uint8_t status;
	uint8_t buf[7];

	/* Bắt đầu một phép đo */
	if (WriteReg(
			dev,
			REG_CONTROL0,
			CTRL0_TM_M) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* Chờ đo xong */
	uint32_t start = HAL_GetTick();

	do
	{
		if (ReadReg(
				dev,
				REG_STATUS,
				&status,
				1) != HAL_OK)
		{
			return HAL_ERROR;
		}

		/* Timeout 10 ms */
		if ((HAL_GetTick() - start) > 10)
		{
			return HAL_TIMEOUT;
		}

	} while ((status & STATUS_MEAS_M_DONE) == 0);

	/* Đọc X, Y, Z và byte 2-bit cuối */
	if (ReadReg(
			dev,
			REG_XOUT0,
			buf,
			7) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/*
	 * Ghép X 18-bit
	 *
	 * X = XOUT0[7:0]
	 *     XOUT1[7:0]
	 *     XYZOUT2[7:6]
	 */
	uint32_t raw_x =
		((uint32_t)buf[0] << 10) |
		((uint32_t)buf[1] << 2) |
		((buf[6] >> 6) & 0x03);

	/*
	 * Ghép Y 18-bit
	 *
	 * Y = YOUT0[7:0]
	 *     YOUT1[7:0]
	 *     XYZOUT2[5:4]
	 */
	uint32_t raw_y =
		((uint32_t)buf[2] << 10) |
		((uint32_t)buf[3] << 2) |
		((buf[6] >> 4) & 0x03);

	/*
	 * Ghép Z 18-bit
	 *
	 * Z = ZOUT0[7:0]
	 *     ZOUT1[7:0]
	 *     XYZOUT2[3:2]
	 */
	uint32_t raw_z =
		((uint32_t)buf[4] << 10) |
		((uint32_t)buf[5] << 2) |
		((buf[6] >> 2) & 0x03);

	/* Trả dữ liệu cho chương trình */
	*x = (int32_t)raw_x;
	*y = (int32_t)raw_y;
	*z = (int32_t)raw_z;

	/* Đồng thời lưu vào struct */
	dev->raw_x = *x;
	dev->raw_y = *y;
	dev->raw_z = *z;

	return HAL_OK;
}

/* =========================================================
 * Đọc từ trường và đổi sang Gauss
 *
 * dev : cảm biến
 * x   : lưu từ trường X (Gauss)
 * y   : lưu từ trường Y (Gauss)
 * z   : lưu từ trường Z (Gauss)
 * ========================================================= */
HAL_StatusTypeDef MMC5983_ReadMag(
	MMC5983MA_t *dev,
	float *x,
	float *y,
	float *z)
{
	int32_t raw_x;
	int32_t raw_y;
	int32_t raw_z;

	/* Đọc dữ liệu raw */
	if (MMC5983_ReadRaw(
			dev,
			&raw_x,
			&raw_y,
			&raw_z) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/*
	 * MMC5983MA:
	 *
	 * Zero-field = 131072
	 * Sensitivity = 16384 counts/Gauss
	 */
	*x = ((float)raw_x - 131072.0f) / 16384.0f;
	*y = ((float)raw_y - 131072.0f) / 16384.0f;
	*z = ((float)raw_z - 131072.0f) / 16384.0f;

	/* Lưu vào struct */
	dev->mag_x = *x;
	dev->mag_y = *y;
	dev->mag_z = *z;

	return HAL_OK;
}