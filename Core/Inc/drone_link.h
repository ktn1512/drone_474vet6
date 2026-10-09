#ifndef DRONE_LINK_H
#define DRONE_LINK_H

#include "e220.h"
#include "angle_controller.h"
#include "rate_controller.h"
#include <stdbool.h>
#include <stdint.h>

#define DRONE_LINK_TIMEOUT_MS 800U
#define DRONE_LINK_MIN_THR 1000U
#define DRONE_LINK_MAX_THR 1700U

// Protocol: ASCII line with XOR checksum *HH before '\n'.
// T,seq,throttle,arm ; P,seq,id,kp,ki,kd ; G,seq,id ; Q,seq
// A,seq,id ; N,seq,id ; R,seq,id,kp,ki,kd ; S,throttle,armed,roll,pitch,yaw,m1,m2,m3,m4

typedef struct {
    E220_t *radio;
    AngleController_t *angle;
    RateController_t *rate;
    char line[112];
    uint8_t tx_buffer[160]; // persistent buffer for non-blocking UART TX
    uint8_t line_len;
    uint16_t throttle_us;
    uint8_t armed;
    uint32_t last_throttle_ms;
} DroneLink_t;

void DroneLink_Init(DroneLink_t *link, E220_t *radio,
                    AngleController_t *angle, RateController_t *rate);
void DroneLink_Poll(DroneLink_t *link);
void DroneLink_ForceDisarm(DroneLink_t *link);
uint16_t DroneLink_Throttle(const DroneLink_t *link);
bool DroneLink_Armed(const DroneLink_t *link);
void DroneLink_SendStatus(DroneLink_t *link, float roll, float pitch, float yaw,
                          uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif
