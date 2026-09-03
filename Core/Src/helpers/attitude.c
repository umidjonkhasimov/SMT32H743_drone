#include "helpers/attitude.h"
#include "helpers/filter.h"
#include <math.h>

#define RAD_TO_DEG 57.2957795f

/* Gravity's direction changes at a few Hz even in aggressive flight, while
   prop vibration lives well above 100 Hz. Filtering hard here costs almost
   nothing real and removes the noise that would otherwise make the
   magnitude test below reject good samples. */
#define ACCEL_LPF_HZ   10.0f

/* How far the filtered acceleration may stray from 1 g before we stop
   believing it points at the centre of the earth. */
#define ACCEL_TRUST_LO 0.80f
#define ACCEL_TRUST_HI 1.20f

static Attitude_t s_att;
static float      s_tau = 1.0f;
static bool       s_trusted;
static bool       s_seeded;
static PT1_t      s_accelLpf[3];

static uint32_t   s_trustedCount;
static uint32_t   s_totalCount;

void Attitude_Init(float tau, float dt)
{
    s_tau       = tau;
    s_att.roll  = 0.0f;
    s_att.pitch = 0.0f;
    s_trusted   = false;
    s_seeded    = false;

    s_trustedCount = 0;
    s_totalCount   = 0;

    for (int i = 0; i < 3; i++)
    {
        PT1_Init(&s_accelLpf[i], ACCEL_LPF_HZ, dt);
    }
}

void Attitude_Update(const IMU_Rates_t *rates, const float accel[3], float dt)
{
    /* Integrating the gyro gives a clean, fast angle that drifts without
       bound. The accelerometer gives a noisy, lag-prone angle that never
       drifts, because gravity does not move. A complementary filter takes
       the short-term answer from the first and the long-term answer from
       the second. */
    s_att.roll  += rates->roll  * dt;
    s_att.pitch += rates->pitch * dt;

    /* Filter before anything else. Testing the raw magnitude means prop
       vibration alone can push a perfectly good sample outside the trust
       band, and an estimate that rejects most of its corrections is just
       an integrating gyro - it drifts, and angle mode drifts with it. */
    const float ax = PT1_Apply(&s_accelLpf[0], accel[0]);   /* forward */
    const float ay = PT1_Apply(&s_accelLpf[1], accel[1]);   /* right   */
    const float az = PT1_Apply(&s_accelLpf[2], accel[2]);   /* down    */

    const float mag = sqrtf(ax * ax + ay * ay + az * az);

    /* During a hard climb or a sharp turn the accelerometer measures thrust
       as well as gravity, and the angle it implies is genuinely wrong. Skip
       those and coast on the gyro - over a second or two of manoeuvring the
       drift is small. */
    s_trusted = (mag > ACCEL_TRUST_LO && mag < ACCEL_TRUST_HI);

    s_totalCount++;

    if (!s_trusted)
    {
        return;
    }

    s_trustedCount++;

    /* At rest and level the body-frame accelerometer reads (0, 0, -1 g):
       it measures the reaction to gravity, which points up, and body Z
       points down. */
    const float accRoll  = atan2f(-ay, -az) * RAD_TO_DEG;
    const float accPitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    if (!s_seeded)
    {
        /* Start from the accelerometer rather than from zero, so the craft
           does not spend the first few seconds believing it is level when
           it is sitting on a slope. */
        s_att.roll  = accRoll;
        s_att.pitch = accPitch;
        s_seeded    = true;

        return;
    }

    const float alpha = s_tau / (s_tau + dt);

    s_att.roll  = alpha * s_att.roll  + (1.0f - alpha) * accRoll;
    s_att.pitch = alpha * s_att.pitch + (1.0f - alpha) * accPitch;
}

Attitude_t Attitude_Get(void)          { return s_att; }
bool       Attitude_AccelTrusted(void) { return s_trusted; }

float Attitude_TrustRatio(void)
{
    if (s_totalCount == 0u)
    {
        return 0.0f;
    }

    return (float)s_trustedCount / (float)s_totalCount;
}

void Attitude_ResetTrustStats(void)
{
    s_trustedCount = 0;
    s_totalCount   = 0;
}
