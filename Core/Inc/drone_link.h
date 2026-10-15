/**
 * @file drone_link.h
 * @brief Giao thức ASCII điều khiển throttle trực tiếp, đọc/ghi PID và trả telemetry.
 * @details Payload kết thúc bằng checksum XOR *HH và newline. Không có ARM/DISARM.
 * Throttle hợp lệ yêu cầu một bản tin S; PID dùng một ô trả lời chờ, ưu tiên hơn S.
 * Gửi thành công chỉ chứng minh HAL chấp nhận TX UART, không xác nhận nhận qua RF.
 */
#ifndef DRONE_LINK_H
#define DRONE_LINK_H

#include "e220.h"
#include "angle_controller.h"
#include "rate_controller.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief Throttle tối thiểu và giá trị khi mất lệnh quá hạn, đơn vị us. */
#define DRONE_LINK_MIN_THR 1000U
/** @brief Throttle tối đa được chấp nhận từ khung T, đơn vị us. */
#define DRONE_LINK_MAX_THR 1700U
/** @brief Ngưỡng tuổi lệnh T, ms; chỉ quá hạn khi tuổi lớn hơn giá trị này. */
#define DRONE_LINK_TIMEOUT_MS 4500U

/** @brief Rate-pitch sample and extrema collected until a status is sent. */
typedef struct {
    uint32_t window_ms;
    uint32_t samples;
    uint32_t invalid_pid_samples;
    float gyro_y;
    float gyro_y_min;
    float gyro_y_max;
    float rate_sp;
    float p_term;
    float i_term;
    float d_term;
    float d_min;
    float d_max;
    float output;
    float dt_ms;
    float dt_min_ms;
    float dt_max_ms;
} DroneLink_PitchDebug_t;

/* Protocol no ARM/DISARM, ASCII XOR *HH\n:
 * ESP32 -> STM32: T,seq,throttle | G,seq,id | P,seq,id,kp,ki,kd
 * STM32 -> ESP32: S,seq,throttle,roll,pitch,yaw,m1,m2,m3,m4
 *                  R,seq,id,kp,ki,kd | A,seq,id | N,seq,id
 * PID ids 0 Angle Roll, 1 Angle Pitch, 2 Rate Roll, 3 Rate Pitch, 4 Rate Yaw.
 */
/**
 * @brief Bộ phân tích dòng, trạng thái throttle và các ô phản hồi chờ của DroneLink.
 * @details Vòng lặp chính dùng đối tượng này; RX ISR thuộc driver E220.
 * reply_payload chỉ chứa một phản hồi, không phải FIFO; yêu cầu PID tiếp theo
 * có thể ghi đè phản hồi chưa gửi. Các yêu cầu status gộp lại và giữ seq T mới nhất.
 * Bộ đếm uint32_t có thể tràn tự nhiên; valid frame không đồng nghĩa PID đã được ghi.
 */
typedef struct {
    E220_t *radio; /**< Driver E220 RX FIFO và TX DMA được link sử dụng. */
    AngleController_t *angle; /**< Controller góc; id 0 là roll, id 1 là pitch. */
    RateController_t *rate; /**< Controller tốc độ; id 2 roll, id 3 pitch, id 4 yaw. */
    char line[128]; /**< Bộ đệm khung RX, tối đa 127 byte trước NUL; chưa gồm LF và CR bị bỏ. */
    uint16_t line_len; /**< Số byte hiện được ghép trong line, chưa gồm NUL. */
    bool rx_discard; /**< true khi bỏ phần còn lại của dòng lỗi đến LF tiếp theo. */
    uint16_t throttle_us; /**< Throttle T hợp lệ đang lưu, us; khởi tạo ở mức tối thiểu. */
    uint32_t last_throttle_ms; /**< Tick HAL lúc khởi tạo hoặc nhận T hợp lệ gần nhất, ms. */

    /* Pending response is transmitted by DMA only when UART is free. */
    char reply_payload[128]; /**< Một payload R/A/N chờ gửi, có NUL, chưa gồm *HH và LF. */
    bool reply_pending; /**< true khi có phản hồi PID chưa được HAL chấp nhận TX DMA. */
    bool status_pending; /**< true khi cần trả S cho T gần nhất; PID nhận sau đó xóa cờ này. */
    uint32_t status_seq; /**< Sequence của T hợp lệ mới nhất cần phản hồi S; không kiểm tra thứ tự seq. */

    uint32_t rx_byte_count; /**< Số byte link lấy khỏi FIFO E220, kể cả CR/LF và byte của dòng bị bỏ. */
    uint32_t rx_line_count; /**< Số ký tự LF đã nhận, kể cả dòng rỗng hoặc đang bỏ. */
    uint32_t rx_valid_frames; /**< Số dòng handle_line chấp nhận, bao gồm P được trả N vì PID/gain không hợp lệ. */
    uint32_t rx_invalid_frames; /**< Số dòng xử lý lỗi hoặc lần bắt đầu bỏ dòng quá dài/chứa NUL. */
    uint32_t throttle_rx_count; /**< Số khung T hợp lệ được áp dụng. */
    uint32_t throttle_timeout_count; /**< Số lần Poll hạ throttle khác mức tối thiểu vì quá hạn T. */
    uint32_t pid_write_count; /**< Số khung P có id/gain hợp lệ đã ghi PID và reset trạng thái PID. */
    uint32_t pid_read_count; /**< Số khung G hợp lệ đã tạo phản hồi R, chưa xác nhận truyền. */
    uint32_t status_request_count; /**< Số yêu cầu S tạo từ T hợp lệ, kể cả yêu cầu bị gộp/ghi đè. */
    uint32_t tx_ok_count; /**< Số gói HAL chấp nhận TX DMA, không phải số gói nhận qua RF. */
    uint32_t tx_busy_count; /**< Số lần thử gửi gặp driver/UART bận; thử lại có thể tăng nhiều lần cho một gói. */
    uint32_t tx_failed_count; /**< Số lỗi tạo gói trong send_payload hoặc lỗi HAL khác HAL_BUSY. */
} DroneLink_t;

/**
 * @brief Xóa trạng thái link, liên kết radio/controller và đặt throttle tối thiểu.
 * @param[out] link Đối tượng link cần khởi tạo, phải khác NULL.
 * @param[in] radio Driver E220 đã khởi tạo, có UART và DMA TX hợp lệ.
 * @param[in] angle Controller góc đã khởi tạo, dùng cho PID id 0 và 1.
 * @param[in] rate Controller tốc độ đã khởi tạo, dùng cho PID id 2, 3 và 4.
 * @note Các đối tượng được trỏ tới phải tồn tại suốt thời gian dùng link.
 * Hàm chỉ lưu con trỏ controller, không khởi tạo lại các controller.
 */
void DroneLink_Init(DroneLink_t *link, E220_t *radio,
                    AngleController_t *angle, RateController_t *rate);
/**
 * @brief Áp timeout throttle, tiêu thụ FIFO RX, phân tích dòng và thử gửi phản hồi PID.
 * @param[in,out] link Link hợp lệ; cập nhật parser, throttle, PID, ô chờ và bộ đếm.
 * @note Gọi thường xuyên trong vòng lặp chính. CR bị bỏ qua; NUL hoặc dòng dài
 * quá 127 byte làm bỏ phần còn lại đến LF. Chỉ lệnh T hợp lệ làm mới timeout.
 * PID được ưu tiên; HAL_BUSY giữ phản hồi chờ để thử lại ở lần poll sau.
 */
void DroneLink_Poll(DroneLink_t *link);
/**
 * @brief Đặt throttle đang lưu về DRONE_LINK_MIN_THR.
 * @param[in,out] link Link hợp lệ có throttle cần đưa về mức tối thiểu.
 * @note Không làm mới last_throttle_ms, không tăng bộ đếm và không xóa ô chờ.
 */
void DroneLink_SetMinimumThrottle(DroneLink_t *link);
/**
 * @brief Lấy throttle có xét tuổi lệnh T gần nhất, không đổi trạng thái link.
 * @param[in] link Link hợp lệ đã khởi tạo.
 * @return Throttle đơn vị us; DRONE_LINK_MIN_THR khi tuổi lệnh lớn hơn
 * DRONE_LINK_TIMEOUT_MS, ngược lại trả throttle_us đang lưu.
 * @note Chênh lệch tick được tính theo uint32_t để xử lý vòng tick HAL.
 */
uint16_t DroneLink_Throttle(const DroneLink_t *link);
/**
 * @brief Thử gửi S cho lệnh T hợp lệ gần nhất khi không có phản hồi PID chờ.
 * @param[in,out] link Link hợp lệ; cập nhật cờ status và bộ đếm TX nếu thử gửi.
 * @param[in] roll Góc roll hiện tại, độ.
 * @param[in] pitch Góc pitch hiện tại, độ.
 * @param[in] yaw Góc yaw hiện tại, độ.
 * @param[in] m1 Độ rộng xung điều khiển động cơ 1, us.
 * @param[in] m2 Độ rộng xung điều khiển động cơ 2, us.
 * @param[in] m3 Độ rộng xung điều khiển động cơ 3, us.
 * @param[in] m4 Độ rộng xung điều khiển động cơ 4, us.
 * @retval true HAL đã chấp nhận TX DMA và ô status chờ đã được xóa.
 * @retval false Không có yêu cầu, đang chờ phản hồi PID, UART bận hoặc lỗi tạo/gửi gói.
 * @note Góc được định dạng một chữ số thập phân. false không xóa status_pending;
 * true chưa xác nhận hoàn tất UART hay nhận RF.
 */
bool DroneLink_SendStatus(DroneLink_t *link, float roll, float pitch, float yaw,
                          uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
/**
 * @brief Send one S with optional rate-pitch diagnostic suffix over E220 DMA.
 * @details S prefix is unchanged. Optional suffix:
 * ,D,w,n,bad,gy,gyMin,gyMax,sp,P,I,D,DMin,DMax,out,dt_ms,dtMin_ms,dtMax_ms
 * gy/sp use deg/s, PID terms use us, w/dt use ms. Floats use 3 significant digits.
 * Caller retains its extrema until true; PID replies retain priority.
 */
bool DroneLink_SendStatusDebug(DroneLink_t *link, float roll, float pitch, float yaw,
                              uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4,
                              const DroneLink_PitchDebug_t *debug);
#endif
