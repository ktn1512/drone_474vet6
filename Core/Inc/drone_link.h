#ifndef DRONE_LINK_H
#define DRONE_LINK_H

#include "e220.h"
#include "angle_controller.h"
#include "rate_controller.h"
#include <stdbool.h>
#include <stdint.h>

#define DRONE_LINK_MIN_THR 1000U
#define DRONE_LINK_MAX_THR 1700U
#define DRONE_LINK_TIMEOUT_MS 4500U

/* Protocol no ARM/DISARM, ASCII XOR *HH\n:
 * ESP32 -> STM32: T,seq,throttle | G,seq,id | P,seq,id,kp,ki,kd
 * STM32 -> ESP32: S,seq,throttle,roll,pitch,yaw,m1,m2,m3,m4
 *                  R,seq,id,kp,ki,kd | A,seq,id | N,seq,id
 * PID ids 0 Angle Roll, 1 Angle Pitch, 2 Rate Roll, 3 Rate Pitch, 4 Rate Yaw.
 */
typedef struct {
    E220_t *radio;
    AngleController_t *angle;
    RateController_t *rate;
    char line[128];
    uint16_t line_len;
    bool rx_discard;
    uint16_t throttle_us;
    uint32_t last_throttle_ms;

    /* Pending response is transmitted by DMA only when UART is free. */
    char reply_payload[128];
    bool reply_pending;
    bool status_pending;
    uint32_t status_seq;

    uint32_t rx_byte_count;
    uint32_t rx_line_count;
    uint32_t rx_valid_frames;
    uint32_t rx_invalid_frames;
    uint32_t throttle_rx_count;
    uint32_t throttle_timeout_count;
    uint32_t pid_write_count;
    uint32_t pid_read_count;
    uint32_t status_request_count;
    uint32_t tx_ok_count;
    uint32_t tx_busy_count;
    uint32_t tx_failed_count;
} DroneLink_t;

void DroneLink_Init(DroneLink_t *link, E220_t *radio,
                    AngleController_t *angle, RateController_t *rate);
void DroneLink_Poll(DroneLink_t *link);
void DroneLink_SetMinimumThrottle(DroneLink_t *link);
uint16_t DroneLink_Throttle(const DroneLink_t *link);
bool DroneLink_SendStatus(DroneLink_t *link, float roll, float pitch, float yaw,
                          uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
#endif
