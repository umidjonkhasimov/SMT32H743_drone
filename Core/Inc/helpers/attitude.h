#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "helpers/imu.h"
#include <stdbool.h>

/* Absolute orientation relative to level, in degrees. Sign conventions match
   the rate controller: positive roll = right side down, positive pitch =
   nose up. There is no yaw here - the accelerometer cannot observe rotation
   about the gravity vector, so yaw stays rate-controlled. */
typedef struct
{
    float roll;
    float pitch;
} Attitude_t;

/* tau is the complementary filter time constant in seconds: roughly how long
   the estimate leans on the gyro before the accelerometer pulls it back. */
void Attitude_Init(float tau, float dt);

void Attitude_Update(const IMU_Rates_t *rates, const float accel[3], float dt);

Attitude_t Attitude_Get(void);

/* False while the accelerometer is being ignored because the craft is
   accelerating hard enough that it is not measuring gravity alone. */
bool Attitude_AccelTrusted(void);

/* Fraction of samples where the accelerometer was believed, since the last
   reset. In steady flight this should be most of them - a low number means
   the estimate is coasting on the gyro and will drift. */
float Attitude_TrustRatio(void);
void  Attitude_ResetTrustStats(void);

#endif /* ATTITUDE_H */
