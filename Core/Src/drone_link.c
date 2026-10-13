#include "drone_link.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t xor_checksum(const char *data) {
    uint8_t checksum = 0U;
    while (*data) checksum ^= (uint8_t)*data++;
    return checksum;
}

/* E220_Send_DMA copies the whole packet into an internal persistent buffer,
 * so the local `packet` array below is safe even after function returns. */
static bool send_payload(DroneLink_t *l, const char *payload) {
    char packet[160];
    if (l->radio->tx_active || l->radio->huart->gState != HAL_UART_STATE_READY) {
        ++l->tx_busy_count;
        return false;
    }
    int n = snprintf(packet, sizeof(packet), "%s*%02X\n", payload,
                     (unsigned)xor_checksum(payload));
    if (n <= 0 || n >= (int)sizeof(packet)) {
        ++l->tx_failed_count;
        return false;
    }
    HAL_StatusTypeDef rc = E220_Send_DMA(l->radio, (const uint8_t *)packet, (uint16_t)n);
    if (rc == HAL_OK) { ++l->tx_ok_count; return true; }
    if (rc == HAL_BUSY) ++l->tx_busy_count;
    else ++l->tx_failed_count;
    return false;
}

static bool validate_frame(char *s) {
    size_t n = strlen(s);
    if (n < 4U || s[n - 3U] != '*') return false;
    char hex[3] = {s[n - 2U], s[n - 1U], '\0'};
    char *end = NULL;
    unsigned long checksum = strtoul(hex, &end, 16);
    if (end != hex + 2 || checksum > 255U) return false;
    s[n - 3U] = '\0';
    return xor_checksum(s) == (uint8_t)checksum;
}

static bool parse_u32(const char *s, uint32_t *value) {
    if (!s || !*s || *s == '-') return false;
    char *end = NULL;
    unsigned long n = strtoul(s, &end, 10);
    if (end == s || *end || n > 0xFFFFFFFFUL) return false;
    *value = (uint32_t)n;
    return true;
}

static bool parse_gain(const char *s, float *value) {
    if (!s || !*s) return false;
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || *end || !isfinite(v) || v < 0.0f || v > 100.0f)
        return false;
    *value = v;
    return true;
}

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

static void set_reply(DroneLink_t *l, const char *payload) {
    /* The ESP32 sends one PID request at a time and waits for reply. */
    snprintf(l->reply_payload, sizeof(l->reply_payload), "%s", payload);
    l->reply_pending = true;
}

static bool handle_line(DroneLink_t *l, char *line) {
    if (!validate_frame(line)) return false;
    char *kind = strtok(line, ",");
    char *seq_s = strtok(NULL, ",");
    uint32_t seq;
    if (!kind || !parse_u32(seq_s, &seq)) return false;

    if (strcmp(kind, "T") == 0) {
        uint32_t thr;
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

    uint32_t id;
    if (!parse_u32(strtok(NULL, ","), &id)) return false;
    PID_t *pid = pid_from_id(l, id);
    char reply[128];
    if (kind[0] == 'G') {
        if (strtok(NULL, ",") != NULL || !pid) return false;
        snprintf(reply, sizeof(reply), "R,%lu,%lu,%.4f,%.4f,%.4f",
                 (unsigned long)seq, (unsigned long)id,
                 pid->kp, pid->ki, pid->kd);
        ++l->pid_read_count;
    } else {
        float kp, ki, kd;
        bool valid = parse_gain(strtok(NULL, ","), &kp) &&
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

void DroneLink_Init(DroneLink_t *l, E220_t *radio,
                    AngleController_t *angle, RateController_t *rate) {
    memset(l, 0, sizeof(*l));
    l->radio = radio;
    l->angle = angle;
    l->rate = rate;
    l->throttle_us = DRONE_LINK_MIN_THR;
    l->last_throttle_ms = HAL_GetTick();
}

void DroneLink_SetMinimumThrottle(DroneLink_t *l) { l->throttle_us = DRONE_LINK_MIN_THR; }

uint16_t DroneLink_Throttle(const DroneLink_t *l) {
    if ((uint32_t)(HAL_GetTick() - l->last_throttle_ms) > DRONE_LINK_TIMEOUT_MS)
        return DRONE_LINK_MIN_THR;
    return l->throttle_us;
}

void DroneLink_Poll(DroneLink_t *l) {
    if ((uint32_t)(HAL_GetTick() - l->last_throttle_ms) > DRONE_LINK_TIMEOUT_MS &&
        l->throttle_us != DRONE_LINK_MIN_THR) {
        ++l->throttle_timeout_count;
        DroneLink_SetMinimumThrottle(l);
    }
    while (E220_Available(l->radio) != 0U) {
        char ch = (char)E220_ReadByte(l->radio);
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

bool DroneLink_SendStatus(DroneLink_t *l, float roll, float pitch, float yaw,
                          uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    if (!l->status_pending || l->reply_pending) return false;
    char payload[128];
    int n = snprintf(payload, sizeof(payload),
             "S,%lu,%u,%.1f,%.1f,%.1f,%u,%u,%u,%u",
             (unsigned long)l->status_seq, (unsigned)DroneLink_Throttle(l),
             roll, pitch, yaw, m1, m2, m3, m4);
    if (n <= 0 || n >= (int)sizeof(payload) || !send_payload(l, payload))
        return false;
    l->status_pending = false;
    return true;
}
