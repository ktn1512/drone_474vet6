/* Host test: gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc
 * tests/test_mahony.c Core/Src/mahony.c -lm -o test_mahony.exe
 */
#include "mahony.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected, float tolerance) {
    assert(isfinite(actual));
    assert(fabsf(actual - expected) < tolerance);
}

static void check_pose(float roll, float pitch) {
    const float rad = 0.01745329252f;
    float ax = -sinf(pitch * rad);
    float ay = sinf(roll * rad) * cosf(pitch * rad);
    float az = cosf(roll * rad) * cosf(pitch * rad);
    float r, p, y;
    Mahony_t m;
    assert(Mahony_InitFromAccel(&m, ax, ay, az));
    Mahony_GetEuler(&m, &r, &p, &y);
    near(r, roll, 0.02f);
    near(p, pitch, 0.02f);
    near(y, 0.0f, 0.02f);
    for (int i = 0; i < 1000; ++i) {
        Mahony_Update(&m, 0, 0, 0, ax, ay, az, 0.002f);
    }
    Mahony_GetEuler(&m, &r, &p, &y);
    near(r, roll, 0.02f);
    near(p, pitch, 0.02f);
    near(y, 0.0f, 0.02f);
}

int main(void) {
    check_pose(0, 0);
    check_pose(30, 0);
    check_pose(-45, 0);
    check_pose(0, 35);
    check_pose(0, -50);
    check_pose(25, -35);
    check_pose(-60, 40);
    check_pose(170, 10);

    Mahony_t m;
    float r, p, y;
    /* Reject invalid gravity without resetting a valid tilt. */
    assert(Mahony_InitFromAccel(&m, 0, 0.5f, 0.8660254f));
    assert(!Mahony_InitFromAccel(&m, 0, 0, 0));
    assert(!Mahony_InitFromAccel(&m, NAN, 0, 1));
    assert(!Mahony_InitFromAccel(&m, 0, INFINITY, 1));
    Mahony_GetEuler(&m, &r, &p, &y);
    near(r, 30, 0.02f);

    /* Gravity must correct an initially wrong estimate within seconds. */
    Mahony_Init(&m);
    for (int i = 0; i < 1500; ++i) {
        Mahony_Update(&m, 0, 0, 0, 0, 0.5f, 0.8660254f, 0.002f);
    }
    Mahony_GetEuler(&m, &r, &p, &y);
    near(r, 30, 0.5f);

    /* Gyro stays in degrees/s at the public interface. */
    Mahony_Init(&m);
    for (int i = 0; i < 500; ++i) {
        Mahony_Update(&m, 0, 0, 90, 0, 0, 1, 0.002f);
    }
    Mahony_GetEuler(&m, &r, &p, &y);
    near(y, 90, 0.02f);

    /* Vertical pitch must remain finite, including roundoff at asin limits. */
    assert(Mahony_InitFromAccel(&m, -1, 0, 0));
    Mahony_GetEuler(&m, &r, &p, &y);
    near(p, 90, 0.05f);
    m.q0 = 0.70710683f;
    m.q1 = 0;
    m.q2 = 0.70710683f;
    m.q3 = 0;
    Mahony_GetEuler(&m, &r, &p, &y);
    near(p, 90, 0.05f);
    puts("Mahony tests passed: startup tilt, stationary hold, invalid input, convergence, gyro units, vertical pitch.");
    return 0;
}
