/**
 * @file drone_link.c
 * @brief Parser khung XOR, throttle có timeout, cập nhật PID và phản hồi qua DMA.
 * @details Không có ARM/DISARM; PID ưu tiên telemetry. Các ô phản hồi chờ
 * là trạng thái một phần tử, không bảo đảm lưu tất cả yêu cầu đến cùng một lần Poll.
 */
#include "drone_link.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief XOR từng byte payload trước dấu sao để tạo checksum 8 bit.
 * @param[in] data Chuỗi hợp lệ kết thúc bằng NUL, không gồm *HH hoặc newline.
 * @return XOR các byte; chuỗi rỗng trả 0.
 */
static uint8_t xor_checksum(const char *data) {
    uint8_t checksum = 0U; /**< Tích lũy XOR 8 bit các byte payload. */
    while (*data) checksum ^= (uint8_t)*data++;
    return checksum;
}

/* E220_Send_DMA copies the whole packet into an internal persistent buffer,
 * so the local `packet` array below is safe even after function returns. */
/**
 * @brief Đóng gói payload thành payload*HH\n và thử khởi động TX DMA.
 * @param[in,out] l Link hợp lệ có radio/UART; cập nhật bộ đếm TX.
 * @param[in] payload Chuỗi hợp lệ có NUL kết thúc, chưa gồm checksum/newline.
 * @retval true E220_Send_DMA trả HAL_OK, dữ liệu đã được sao chép vào driver.
 * @retval false UART bận, gói quá dài hoặc HAL báo lỗi gửi.
 * @note true không xác nhận hoàn tất UART hay thiết bị đầu xa nhận được qua RF.
 */
static bool send_payload(DroneLink_t *l, const char *payload) {
    char packet[E220_MAX_PACKET_SIZE + 1U]; /**< Payload, *HH, LF and terminating NUL. */
    if (l->radio->tx_active || l->radio->huart->gState != HAL_UART_STATE_READY) {
        ++l->tx_busy_count;
        return false;
    }
    int n = snprintf(packet, sizeof(packet), "%s*%02X\n", payload, /**< Số byte gói cần tạo, chưa gồm NUL; dùng kiểm tra cắt chuỗi. */
                     (unsigned)xor_checksum(payload));
    if (n <= 0 || n >= (int)sizeof(packet)) {
        ++l->tx_failed_count;
        return false;
    }
    HAL_StatusTypeDef rc = E220_Send_DMA(l->radio, (const uint8_t *)packet, (uint16_t)n); /**< Kết quả HAL chấp nhận DMA, dùng phân loại bộ đếm TX. */
    if (rc == HAL_OK) { ++l->tx_ok_count; return true; }
    if (rc == HAL_BUSY) ++l->tx_busy_count;
    else ++l->tx_failed_count;
    return false;
}

/**
 * @brief Kiểm tra hậu tố *HH và so checksum XOR của payload.
 * @param[in,out] s Chuỗi có NUL kết thúc, không gồm LF; nếu hậu tố hex hợp lệ,
 * dấu sao bị thay bằng NUL ngay cả khi phép so checksum sau đó thất bại.
 * @retval true Hậu tố được chuyển thành byte và XOR payload trùng checksum.
 * @retval false Khung ngắn, không có dấu sao đúng vị trí, lỗi hex hoặc sai XOR.
 */
static bool validate_frame(char *s) {
    size_t n = strlen(s); /**< Độ dài khung gồm *HH, chưa gồm NUL. */
    if (n < 4U || s[n - 3U] != '*') return false;
    char hex[3] = {s[n - 2U], s[n - 1U], '\0'}; /**< Hai ký tự checksum và NUL kết thúc. */
    char *end = NULL; /**< Vị trí dừng chuyển số, dùng kiểm tra đã đọc hết token. */
    unsigned long checksum = strtoul(hex, &end, 16); /**< Giá trị checksum từ hậu tố hex; chỉ chấp nhận tối đa 255. */
    if (end != hex + 2 || checksum > 255U) return false;
    s[n - 3U] = '\0';
    return xor_checksum(s) == (uint8_t)checksum;
}

/**
 * @brief Chuyển token số nguyên thập phân thành uint32_t nếu toàn token hợp lệ.
 * @param[in] s Token kết thúc bằng NUL; NULL/rỗng hoặc bắt đầu bằng dấu trừ bị từ chối.
 * @param[out] value Con trỏ hợp lệ nhận kết quả; không đổi khi trả false.
 * @retval true strtoul đã đọc hết token và giá trị không vượt 0xFFFFFFFF.
 * @retval false Token không hợp lệ hoặc vượt kiểm tra giới hạn.
 * @note Dấu cộng/khoảng trắng đầu chuỗi tuân theo strtoul; không kiểm tra errno.
 */
static bool parse_u32(const char *s, uint32_t *value) {
    if (!s || !*s || *s == '-') return false;
    char *end = NULL; /**< Vị trí dừng chuyển số, dùng kiểm tra đã đọc hết token. */
    unsigned long n = strtoul(s, &end, 10); /**< Giá trị thập phân trước khi kiểm tra toàn token và miền uint32_t. */
    if (end == s || *end || n > 0xFFFFFFFFUL) return false;
    *value = (uint32_t)n;
    return true;
}

/**
 * @brief Chuyển một token hệ số PID hữu hạn trong khoảng 0..100.
 * @param[in] s Token float có NUL kết thúc, dùng cú pháp strtof.
 * @param[out] value Con trỏ hợp lệ nhận hệ số; không đổi nếu thất bại.
 * @retval true Token được đọc hết, hữu hạn và nằm trong khoảng cho phép.
 * @retval false Token NULL/rỗng, lỗi cú pháp, không hữu hạn hoặc ngoài khoảng.
 * @note Đơn vị hệ số phụ thuộc PID góc hoặc PID tốc độ được chọn; không đổi đơn vị.
 */
static bool parse_gain(const char *s, float *value) {
    if (!s || !*s) return false;
    char *end = NULL; /**< Vị trí dừng chuyển số, dùng kiểm tra đã đọc hết token. */
    float v = strtof(s, &end); /**< Hệ số đang kiểm tra hữu hạn và miền 0..100. */
    if (end == s || *end || !isfinite(v) || v < 0.0f || v > 100.0f)
        return false;
    *value = v;
    return true;
}

/**
 * @brief Ánh xạ id giao thức sang PID trong controller góc hoặc controller tốc độ.
 * @param[in] l Link có angle và rate hợp lệ.
 * @param[in] id Mã PID: 0 góc roll, 1 góc pitch, 2 tốc độ roll, 3 tốc độ pitch, 4 yaw.
 * @return Con trỏ PID tương ứng; NULL nếu id không thuộc 0..4.
 */
static PID_t *pid_from_id(DroneLink_t *l, uint32_t id) {
    switch (id) {
    case 0: return &l->angle->roll;
    case 1: return &l->angle->pitch;
    case 2: return &l->rate->roll;
    case 3: return &l->rate->pitch;
    case 4: return &l->rate->yaw;
    default: return NULL;
    }
}

/**
 * @brief Chép payload vào một ô phản hồi PID và đánh dấu đang chờ gửi.
 * @param[in,out] l Link hợp lệ; thay reply_payload và đặt reply_pending.
 * @param[in] payload Chuỗi có NUL, chưa gồm checksum/newline.
 * @note Ghi đè ô cũ nếu chưa gửi; snprintf cắt payload khi vượt dung lượng.
 */
static void set_reply(DroneLink_t *l, const char *payload) {
    /* The ESP32 sends one PID request at a time and waits for reply. */
    snprintf(l->reply_payload, sizeof(l->reply_payload), "%s", payload);
    l->reply_pending = true;
}

/**
 * @brief Xác thực và xử lý T, G hoặc P; cập nhật throttle/PID và chuẩn bị phản hồi.
 * @param[in,out] l Link hợp lệ; T làm mới timeout, P hợp lệ ghi và reset PID.
 * @param[in,out] line Khung có NUL, chưa gồm LF; bị sửa bởi kiểm checksum và strtok.
 * @retval true T/G hợp lệ hoặc P có seq/id đọc được đã được xử lý, kể cả P trả N.
 * @retval false Sai checksum, lệnh/seq/throttle/id không đọc được hoặc G không hợp lệ.
 * @note PID được phản hồi bằng R/A/N và hủy status đang chờ; không gọi DMA ở đây.
 * Không có kiểm tra seq tăng dần, chống lặp hay ARM/DISARM.
 */
static bool handle_line(DroneLink_t *l, char *line) {
    if (!validate_frame(line)) return false;
    char *kind = strtok(line, ","); /**< Token lệnh T, G hoặc P; trỏ vào line sau khi bị tách. */
    char *seq_s = strtok(NULL, ","); /**< Token sequence cần chuyển thành uint32_t. */
    uint32_t seq; /**< Sequence để phản hồi đúng yêu cầu; không dùng chống lặp. */
    if (!kind || !parse_u32(seq_s, &seq)) return false;

    if (strcmp(kind, "T") == 0) {
        uint32_t thr; /**< Throttle đọc từ T, đơn vị us; phải nằm trong 1000..1700. */
        if (!parse_u32(strtok(NULL, ","), &thr) || strtok(NULL, ",") != NULL ||
            thr < DRONE_LINK_MIN_THR || thr > DRONE_LINK_MAX_THR) return false;
        l->throttle_us = (uint16_t)thr;
        l->last_throttle_ms = HAL_GetTick();
        ++l->throttle_rx_count;
        l->status_seq = seq;
        l->status_pending = true;
        ++l->status_request_count;
        return true;
    }
    if (strcmp(kind, "P") != 0 && strcmp(kind, "G") != 0) return false;

    uint32_t id; /**< Id PID do ESP32 yêu cầu, quy ước 0..4. */
    if (!parse_u32(strtok(NULL, ","), &id)) return false;
    PID_t *pid = pid_from_id(l, id); /**< PID được chọn hoặc NULL nếu id ngoài quy ước. */
    char reply[128]; /**< Payload R/A/N tạm trước khi chép vào ô phản hồi chờ. */
    if (kind[0] == 'G') {
        if (strtok(NULL, ",") != NULL || !pid) return false;
        snprintf(reply, sizeof(reply), "R,%lu,%lu,%.4f,%.4f,%.4f",
                 (unsigned long)seq, (unsigned long)id,
                 pid->kp, pid->ki, pid->kd);
        ++l->pid_read_count;
    } else {
        float kp, ki, kd; /**< Hệ số tỉ lệ, tích phân và vi phân; đơn vị theo PID được chọn. */
        bool valid = parse_gain(strtok(NULL, ","), &kp) && /**< Cờ cả ba gain hợp lệ và không còn token dư. */
                     parse_gain(strtok(NULL, ","), &ki) &&
                     parse_gain(strtok(NULL, ","), &kd) &&
                     strtok(NULL, ",") == NULL;
        if (valid && pid) {
            pid->kp = kp; pid->ki = ki; pid->kd = kd;
            PID_Reset(pid);
            ++l->pid_write_count;
            snprintf(reply, sizeof(reply), "A,%lu,%lu",
                     (unsigned long)seq, (unsigned long)id);
        } else {
            snprintf(reply, sizeof(reply), "N,%lu,%lu",
                     (unsigned long)seq, (unsigned long)id);
        }
    }
    /* Respond to PID first. Do not send an old telemetry frame ahead of ACK. */
    l->status_pending = false;
    set_reply(l, reply);
    return true;
}

/** @brief Xóa parser/bộ đếm, gán các đối tượng và bắt đầu mốc timeout throttle. */
void DroneLink_Init(DroneLink_t *l, E220_t *radio,
                    AngleController_t *angle, RateController_t *rate) {
    memset(l, 0, sizeof(*l));
    l->radio = radio;
    l->angle = angle;
    l->rate = rate;
    l->throttle_us = DRONE_LINK_MIN_THR;
    l->last_throttle_ms = HAL_GetTick();
}

/** @brief Đặt riêng giá trị throttle đang lưu về mức tối thiểu. */
void DroneLink_SetMinimumThrottle(DroneLink_t *l) { l->throttle_us = DRONE_LINK_MIN_THR; }

/** @brief Trả throttle với điều kiện tuổi T không lớn hơn ngưỡng timeout. */
uint16_t DroneLink_Throttle(const DroneLink_t *l) {
    if ((uint32_t)(HAL_GetTick() - l->last_throttle_ms) > DRONE_LINK_TIMEOUT_MS)
        return DRONE_LINK_MIN_THR;
    return l->throttle_us;
}

/** @brief Xử lý timeout, ghép toàn bộ byte RX đang có và thử gửi phản hồi PID chờ. */
void DroneLink_Poll(DroneLink_t *l) {
    if ((uint32_t)(HAL_GetTick() - l->last_throttle_ms) > DRONE_LINK_TIMEOUT_MS &&
        l->throttle_us != DRONE_LINK_MIN_THR) {
        ++l->throttle_timeout_count;
        DroneLink_SetMinimumThrottle(l);
    }
    while (E220_Available(l->radio) != 0U) {
        char ch = (char)E220_ReadByte(l->radio); /**< Byte RX vừa lấy; LF kết thúc dòng, CR bị bỏ. */
        ++l->rx_byte_count;
        if (ch == '\n') {
            ++l->rx_line_count;
            if (!l->rx_discard && l->line_len != 0U) {
                l->line[l->line_len] = '\0';
                if (handle_line(l, l->line)) ++l->rx_valid_frames;
                else ++l->rx_invalid_frames;
            }
            l->line_len = 0U;
            l->rx_discard = false;
        } else if (ch != '\r' && !l->rx_discard) {
            if (ch != '\0' && l->line_len < sizeof(l->line) - 1U)
                l->line[l->line_len++] = ch;
            else {
                l->line_len = 0U;
                l->rx_discard = true;
                ++l->rx_invalid_frames;
            }
        }
    }
    if (l->reply_pending && send_payload(l, l->reply_payload))
        l->reply_pending = false;
}

/** @brief Tạo S theo seq T mới nhất và gửi DMA nếu không có phản hồi PID ưu tiên. */
bool DroneLink_SendStatus(DroneLink_t *l, float roll, float pitch, float yaw,
                          uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    return DroneLink_SendStatusDebug(l, roll, pitch, yaw, m1, m2, m3, m4, NULL);
}

bool DroneLink_SendStatusDebug(DroneLink_t *l, float roll, float pitch, float yaw,
                              uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4,
                              const DroneLink_PitchDebug_t *debug) {
    if (!l->status_pending || l->reply_pending) return false;
    /* Avoid repeated float formatting while a previous DMA transfer is active. */
    if (l->radio->tx_active || l->radio->huart->gState != HAL_UART_STATE_READY) {
        ++l->tx_busy_count;
        return false;
    }
    char payload[E220_MAX_PACKET_SIZE - 3U]; /**< Leave four bytes for *HH and LF. */
    int n = snprintf(payload, sizeof(payload), /**< Số byte S cần tạo, chưa gồm NUL; kiểm tra tràn bộ đệm định dạng. */
             "S,%lu,%u,%.1f,%.1f,%.1f,%u,%u,%u,%u",
             (unsigned long)l->status_seq, (unsigned)DroneLink_Throttle(l),
             roll, pitch, yaw, m1, m2, m3, m4);
    if (n <= 0 || n >= (int)sizeof(payload)) return false;
    if (debug != NULL && debug->samples != 0U) {
        int added = snprintf(payload + n, sizeof(payload) - (size_t)n,
            ",D,%lu,%lu,%lu,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g,%.3g",
            (unsigned long)debug->window_ms, (unsigned long)debug->samples,
            (unsigned long)debug->invalid_pid_samples,
            debug->gyro_y, debug->gyro_y_min, debug->gyro_y_max, debug->rate_sp,
            debug->p_term, debug->i_term, debug->d_term, debug->d_min, debug->d_max,
            debug->output, debug->dt_ms, debug->dt_min_ms, debug->dt_max_ms);
        if (added <= 0 || (size_t)added >= sizeof(payload) - (size_t)n) {
            ++l->tx_failed_count;
            return false;
        }
    }
    if (!send_payload(l, payload))
        return false;
    l->status_pending = false;
    return true;
}
