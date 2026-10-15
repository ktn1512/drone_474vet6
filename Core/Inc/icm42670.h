/**
 * @file icm42670.h
 * @brief Giao diện đọc, cấu hình và hiệu chuẩn gyro cho ICM-42670.
 */

/*
 * icm42670.h
 * ICM-42670-P IMU driver
 */

#ifndef INC_ICM42670_H_
#define INC_ICM42670_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

/**
 * @brief Handle SPI1 do mã khởi tạo HAL cung cấp cho cảm biến.
 */
extern SPI_HandleTypeDef hspi1;

/**
 * @brief Handle SPI được dùng trong mọi giao dịch với ICM-42670.
 */
#define ICM42670_SPI        hspi1
/**
 * @brief Cổng GPIO điều khiển chân chip-select của cảm biến.
 */
#define ICM_CS_PORT         GPIOA
/**
 * @brief Chân chip-select SPI, được kích hoạt ở mức thấp.
 */
#define ICM_CS_PIN          GPIO_PIN_4

/* Device registers */
/**
 * @brief Địa chỉ thanh ghi nhận dạng thiết bị WHO_AM_I.
 */
#define ICM_WHO_AM_I        0x75U
/**
 * @brief Địa chỉ thanh ghi điều khiển chế độ nguồn gyro và accel.
 */
#define ICM_PWR_MGMT0       0x1FU
/**
 * @brief Địa chỉ thanh ghi cấu hình thang đo và tốc độ xuất gyro.
 */
#define ICM_GYRO_CONFIG0    0x20U
/**
 * @brief Địa chỉ thanh ghi cấu hình thang đo và tốc độ xuất accel.
 */
#define ICM_ACCEL_CONFIG0   0x21U
/**
 * @brief Địa chỉ thanh ghi cấu hình bộ lọc gyro.
 */
#define ICM_GYRO_CONFIG1    0x23U
/**
 * @brief Địa chỉ thanh ghi cấu hình bộ lọc và lấy trung bình accel.
 */
#define ICM_ACCEL_CONFIG1   0x24U
/**
 * @brief Địa chỉ byte cao nhiệt độ; điểm bắt đầu khối dữ liệu 14 byte.
 */
#define ICM_TEMP_DATA1      0x09U
/**
 * @brief Địa chỉ byte cao dữ liệu gia tốc trục X.
 */
#define ICM_ACCEL_DATA_X1   0x0BU
/**
 * @brief Địa chỉ byte cao dữ liệu vận tốc góc trục X.
 */
#define ICM_GYRO_DATA_X1    0x11U

/**
 * @brief Giá trị WHO_AM_I mà hàm khởi tạo chấp nhận.
 */
#define ICM_WHO_AM_I_VALUE  0x67U

/* Configuration used by the driver */
/**
 * @brief Biên độ thang đo gyro được cấu hình, đơn vị độ/giây (±).
 */
#define ICM_GYRO_FS_DPS     2000.0f
/**
 * @brief Biên độ thang đo accel được cấu hình, đơn vị g (±).
 */
#define ICM_ACCEL_FS_G      8.0f
/**
 * @brief Hệ số đổi dữ liệu gyro: số LSB trên một độ/giây.
 */
#define ICM_GYRO_LSB_PER_DPS 16.4f
/**
 * @brief Hệ số đổi dữ liệu accel: số LSB trên một g.
 */
#define ICM_ACCEL_LSB_PER_G  4096.0f

/* Init/configuration error codes */
/**
 * @brief Mã kết quả giao dịch SPI, cấu hình và nhận dạng ICM-42670.
 */
typedef enum
{
    /**
     * @brief Thao tác hoàn tất thành công.
     */
    ICM42670_OK = 0,
    /**
     * @brief Tham số không hợp lệ hoặc lỗi SPI khác timeout.
     */
    ICM42670_ERROR = 1,
    /**
     * @brief Giao dịch SPI trả về HAL_TIMEOUT.
     */
    ICM42670_TIMEOUT = 2,
    /**
     * @brief Giá trị WHO_AM_I đọc được khác giá trị mong đợi.
     */
    ICM42670_WHOAMI_ERROR = 3
} ICM42670_Status_t;

/**
 * @brief Một mẫu IMU gồm dữ liệu thô và các giá trị đã đổi đơn vị.
 */
typedef struct
{
    /**
     * @brief Gia tốc thô trục X theo trục cảm biến, đơn vị LSB có dấu.
     */
    int16_t ax_raw;
    /**
     * @brief Gia tốc thô trục Y theo trục cảm biến, đơn vị LSB có dấu.
     */
    int16_t ay_raw;
    /**
     * @brief Gia tốc thô trục Z theo trục cảm biến, đơn vị LSB có dấu.
     */
    int16_t az_raw;

    /**
     * @brief Vận tốc góc thô trục X, đơn vị LSB có dấu.
     */
    int16_t gx_raw;
    /**
     * @brief Vận tốc góc thô trục Y, đơn vị LSB có dấu.
     */
    int16_t gy_raw;
    /**
     * @brief Vận tốc góc thô trục Z, đơn vị LSB có dấu.
     */
    int16_t gz_raw;

    /**
     * @brief Dữ liệu nhiệt độ thô, đơn vị LSB có dấu.
     */
    int16_t temp_raw;

    /**
     * @brief Gia tốc trục X đã đổi từ dữ liệu thô, đơn vị g.
     */
    float ax_g;
    /**
     * @brief Gia tốc trục Y đã đổi từ dữ liệu thô, đơn vị g.
     */
    float ay_g;
    /**
     * @brief Gia tốc trục Z đã đổi từ dữ liệu thô, đơn vị g.
     */
    float az_g;

    /**
     * @brief Vận tốc góc trục X, đơn vị độ/giây; bias chỉ được trừ khi gọi ApplyCalibration.
     */
    float gx_dps;
    /**
     * @brief Vận tốc góc trục Y, đơn vị độ/giây; bias chỉ được trừ khi gọi ApplyCalibration.
     */
    float gy_dps;
    /**
     * @brief Vận tốc góc trục Z, đơn vị độ/giây; bias chỉ được trừ khi gọi ApplyCalibration.
     */
    float gz_dps;

    /**
     * @brief Nhiệt độ tính từ dữ liệu thô, đơn vị °C.
     */
    float temperature_c;

} ICM42670_Data_t;

/* Driver state */
/**
 * @brief Trạng thái khởi tạo và các độ lệch tĩnh của gyro.
 */
typedef struct
{
    /**
     * @brief Bias gyro trục X được lấy trung bình khi hiệu chuẩn, độ/giây.
     */
    float gyro_bias_x_dps;
    /**
     * @brief Bias gyro trục Y được lấy trung bình khi hiệu chuẩn, độ/giây.
     */
    float gyro_bias_y_dps;
    /**
     * @brief Bias gyro trục Z được lấy trung bình khi hiệu chuẩn, độ/giây.
     */
    float gyro_bias_z_dps;

    /**
     * @brief Cờ đặt thành 1 sau khi Init thành công; không được kiểm tra trong các hàm đọc.
     */
    uint8_t initialized;
} ICM42670_t;

/**
 * @brief Xóa trạng thái, kiểm tra ID và cấu hình gyro/accel qua SPI.
 * @param[out] dev Trạng thái cảm biến được khởi tạo; không được NULL.
 * @retval ICM42670_OK Thao tác hoàn tất.
 * @retval ICM42670_TIMEOUT Giao dịch SPI hết thời gian chờ.
 * @retval ICM42670_ERROR Tham số không hợp lệ hoặc lỗi SPI khác.
 * @retval ICM42670_WHOAMI_ERROR ID cảm biến không khớp.
 * @note Hàm có các khoảng HAL_Delay; cờ initialized chỉ được đặt sau toàn bộ cấu hình.
 */
ICM42670_Status_t ICM42670_Init(ICM42670_t *dev);
/**
 * @brief Đọc một byte từ thanh ghi cảm biến.
 * @param[in] reg Địa chỉ thanh ghi, bit đọc SPI được hàm tự thêm.
 * @param[out] data Nơi nhận một byte; không được NULL.
 * @retval ICM42670_OK Thao tác hoàn tất.
 * @retval ICM42670_TIMEOUT Giao dịch SPI hết thời gian chờ.
 * @retval ICM42670_ERROR Tham số không hợp lệ hoặc lỗi SPI khác.
 */
ICM42670_Status_t ICM42670_ReadReg(uint8_t reg, uint8_t *data);
/**
 * @brief Ghi một byte vào thanh ghi cảm biến.
 * @param[in] reg Địa chỉ thanh ghi; bit 7 được xóa khi tạo lệnh ghi.
 * @param[in] data Giá trị byte cần ghi.
 * @retval ICM42670_OK Ghi thành công.
 * @retval ICM42670_TIMEOUT Giao dịch SPI hết thời gian chờ.
 * @retval ICM42670_ERROR Giao dịch SPI thất bại.
 */
ICM42670_Status_t ICM42670_WriteReg(uint8_t reg, uint8_t data);
/**
 * @brief Đọc nhiệt độ, accel XYZ và gyro XYZ dưới dạng dữ liệu thô.
 * @param[out] imu Mẫu nhận các trường *_raw; các trường float giữ nguyên.
 * @retval ICM42670_OK Thao tác hoàn tất.
 * @retval ICM42670_TIMEOUT Giao dịch SPI hết thời gian chờ.
 * @retval ICM42670_ERROR Tham số không hợp lệ hoặc lỗi SPI khác.
 * @note Thứ tự trục giữ nguyên theo cảm biến; không áp dụng bias.
 */
ICM42670_Status_t ICM42670_ReadRaw(ICM42670_Data_t *imu);
/**
 * @brief Đọc mẫu thô và đổi gia tốc, vận tốc góc, nhiệt độ sang đơn vị vật lý.
 * @param[out] imu Mẫu IMU nhận cả trường thô và trường đã đổi đơn vị.
 * @retval ICM42670_OK Thao tác hoàn tất.
 * @retval ICM42670_TIMEOUT Giao dịch SPI hết thời gian chờ.
 * @retval ICM42670_ERROR Tham số không hợp lệ hoặc lỗi SPI khác.
 * @note Accel dùng g, gyro dùng độ/giây, nhiệt độ dùng °C; hàm chưa trừ gyro bias.
 */
ICM42670_Status_t ICM42670_ReadAll(ICM42670_Data_t *imu);

/**
 * @brief Trừ gyro bias đã lưu khỏi các trường vận tốc góc của một mẫu.
 * @param[in] dev Trạng thái chứa bias theo độ/giây.
 * @param[in,out] imu Mẫu có gx_dps/gy_dps/gz_dps đã đổi đơn vị để trừ bias.
 * @note Nếu một con trỏ là NULL, hàm thoát; dữ liệu raw và accel không được cập nhật.
 */
void ICM42670_ApplyCalibration(ICM42670_t *dev, ICM42670_Data_t *imu);
/**
 * @brief Lấy trung bình các mẫu gyro để lưu bias cho ba trục.
 * @param[in,out] dev Trạng thái nhận gyro_bias_*_dps; cờ initialized được giữ nguyên.
 * @param[in] samples Số mẫu cần lấy; phải lớn hơn 0.
 * @param[in] sample_delay_ms Khoảng HAL_Delay sau mỗi mẫu, đơn vị ms; 0 bỏ qua delay.
 * @retval ICM42670_OK Thao tác hoàn tất.
 * @retval ICM42670_TIMEOUT Giao dịch SPI hết thời gian chờ.
 * @retval ICM42670_ERROR Tham số không hợp lệ hoặc lỗi SPI khác.
 * @note Cảm biến cần đứng yên trong quá trình lấy mẫu; hàm không tự phát hiện chuyển động.
 * @note Chỉ cập nhật ba bias sau khi đọc đủ mẫu; không hiệu chỉnh tư thế hay gia tốc.
 */
ICM42670_Status_t ICM42670_CalibrateGyro(ICM42670_t *dev, uint16_t samples, uint16_t sample_delay_ms);

#endif /* INC_ICM42670_H_ */
