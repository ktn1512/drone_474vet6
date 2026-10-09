#include "drone_link.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned char checksum(const char *s) {
    unsigned char c = 0;
    while (*s) c ^= (unsigned char)*s++;
    return c;
}

// Use interrupt TX to keep UART polling out of the 500-Hz PID loop.
// AUX is disconnected: the application must also limit radio traffic.
static bool send_payload(DroneLink_t *l, const char *payload) {
    if (l->radio->huart->gState != HAL_UART_STATE_READY) return false;
    int n = snprintf((char *)l->tx_buffer, sizeof(l->tx_buffer),
                     "%s*%02X\n", payload, (unsigned)checksum(payload));
    if (n <= 0 || n >= (int)sizeof(l->tx_buffer)) return false;
    return E220_Send_IT(l->radio, l->tx_buffer, (uint16_t)n) == HAL_OK;
}

static bool validate_frame(char *line) {
    size_t n = strlen(line);
    if (n < 4U || line[n - 3] != '*') return false;
    char h[3] = { line[n - 2], line[n - 1], 0 };
    char *end = NULL;
    unsigned long check = strtoul(h, &end, 16);
    if (end != h + 2 || check > 255U) return false;
    line[n - 3] = 0;
    return checksum(line) == (unsigned char)check;
}

static bool parse_u32(const char *s, uint32_t *out) {
    if (!s || !*s || *s == '-') return false;
    char *end = NULL;
    unsigned long val = strtoul(s, &end, 10);
    if (end == s || *end != 0 || val > 0xFFFFFFFFUL) return false;
    *out = (uint32_t)val;
    return true;
}

static bool parse_gain(const char *s, float *out) {
    if (!s || !*s) return false;
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || *end != 0 || !isfinite(v) || v < 0.0f || v > 100.0f)
        return false;
    *out = v;
    return true;
}

static PID_t *select_pid(DroneLink_t *l, uint32_t id) {
    switch (id) {
    case 0: return &l->angle->roll;
    case 1: return &l->angle->pitch;
    case 2: return &l->rate->roll;
    case 3: return &l->rate->pitch;
    case 4: return &l->rate->yaw;
    default: return NULL;
    }
}

static void handle_line(DroneLink_t *l, char *line) {
    if (!validate_frame(line)) return;
    // Parser runs only from main loop, never from IRQ. C-standard strtok is safe here.
    char *kind = strtok(line, ",");
    char *seq_s = strtok(NULL, ",");
    uint32_t seq;
    if (!kind || !parse_u32(seq_s, &seq)) return;
    char reply[128];

    if (strcmp(kind, "T") == 0) {
        uint32_t thr, requested_arm;
        char *thr_s = strtok(NULL, ",");
        char *arm_s = strtok(NULL, ",");
        if (!parse_u32(thr_s, &thr) || !parse_u32(arm_s, &requested_arm) ||
            strtok(NULL, ",") != NULL ||
            thr < DRONE_LINK_MIN_THR || thr > DRONE_LINK_MAX_THR || requested_arm > 1)
            return;

        // A fresh, well-formed T updates link freshness; PID/G/Q cannot keep it alive.
        l->last_throttle_ms = HAL_GetTick();
        if (requested_arm == 0) {
            l->armed = 0;
            l->throttle_us = DRONE_LINK_MIN_THR;
        } else if (l->armed || thr == DRONE_LINK_MIN_THR) {
            // Only arm from minimum throttle.
            l->armed = 1;
            l->throttle_us = (uint16_t)thr;
        } else {
            l->armed = 0;
            l->throttle_us = DRONE_LINK_MIN_THR;
        }
    }
    else if (strcmp(kind, "P") == 0 || strcmp(kind, "G") == 0) {
        uint32_t id;
        if (!parse_u32(strtok(NULL, ","), &id)) return;
        PID_t *pid = select_pid(l, id);
        if (!pid) return;
        if (kind[0] == 'G') {
            if (strtok(NULL, ",") != NULL) return;
            snprintf(reply, sizeof(reply), "R,%lu,%lu,%.4f,%.4f,%.4f",
                     (unsigned long)seq, (unsigned long)id,
                     pid->kp, pid->ki, pid->kd);
            send_payload(l, reply);
        } else {
            float kp, ki, kd;
            bool valid = parse_gain(strtok(NULL, ","), &kp) &&
                         parse_gain(strtok(NULL, ","), &ki) &&
                         parse_gain(strtok(NULL, ","), &kd) &&
                         strtok(NULL, ",") == NULL;
            // NEVER allow live retuning while armed. Protect even if web is compromised.
            if (!valid || l->armed) {
                snprintf(reply, sizeof(reply), "N,%lu,%lu",
                         (unsigned long)seq, (unsigned long)id);
            } else {
                pid->kp = kp;
                pid->ki = ki;
                pid->kd = kd;
                PID_Reset(pid);
                snprintf(reply, sizeof(reply), "A,%lu,%lu",
                         (unsigned long)seq, (unsigned long)id);
            }
            send_payload(l, reply);
        }
    }
    else if (strcmp(kind, "Q") == 0) {
        // Status is sent periodically by caller; Q is reserved for future use.
        (void)seq;
    }
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

void DroneLink_ForceDisarm(DroneLink_t *l) {
    l->armed = 0;
    l->throttle_us = DRONE_LINK_MIN_THR;
}

void DroneLink_Poll(DroneLink_t *l) {
    // Check the timeout BEFORE parsing incoming packets. A late high-throttle
    // packet must not revive an armed link after loss of the heartbeat.
    if ((uint32_t)(HAL_GetTick() - l->last_throttle_ms) > DRONE_LINK_TIMEOUT_MS) {
        DroneLink_ForceDisarm(l);
    }
    while (E220_Available(l->radio) != 0) {
        char c = (char)E220_ReadByte(l->radio);
        if (c == '\n') {
            l->line[l->line_len] = 0;
            if (l->line_len > 0) handle_line(l, l->line);
            l->line_len = 0;
        } else if (c != '\r') {
            if (l->line_len < sizeof(l->line) - 1U)
                l->line[l->line_len++] = c;
            else
                l->line_len = 0; // oversize: drop/reset partial line
        }
    }
    if ((uint32_t)(HAL_GetTick() - l->last_throttle_ms) > DRONE_LINK_TIMEOUT_MS) {
        l->armed = 0;
        l->throttle_us = DRONE_LINK_MIN_THR;
    }
}

uint16_t DroneLink_Throttle(const DroneLink_t *l) {
    if (!DroneLink_Armed(l)) return DRONE_LINK_MIN_THR;
    return l->throttle_us;
}

bool DroneLink_Armed(const DroneLink_t *l) {
    return l->armed &&
           (uint32_t)(HAL_GetTick() - l->last_throttle_ms) <= DRONE_LINK_TIMEOUT_MS;
}

void DroneLink_SendStatus(DroneLink_t *l, float roll, float pitch, float yaw,
                          uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    char payload[124];
    int n = snprintf(payload, sizeof(payload),
        "S,%u,%u,%.1f,%.1f,%.1f,%u,%u,%u,%u",
        (unsigned)DroneLink_Throttle(l), (unsigned)DroneLink_Armed(l),
        roll, pitch, yaw, m1, m2, m3, m4);
    if (n > 0 && n < (int)sizeof(payload)) send_payload(l, payload);
}
