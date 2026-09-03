#ifndef IMU_H
#define IMU_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Every ICM-42688-P answers with this in its WHO_AM_I register. */
#define ICM42688_WHOAMI     0x47u

/* Largest burst read we ever do in one transaction (temperature + accel +
   gyro is 14 bytes). Bounds the transfer buffers in imu.c. */
#define IMU_MAX_BURST       16u

/* Configured output data rate, and the filter cutoff derived from it. */
#define IMU_SAMPLE_RATE_HZ  1000.0f
#define IMU_GYRO_LPF_HZ     100.0f

typedef enum
{
    IMU_OK = 0,
    IMU_ERR_SPI,      /* the SPI transfer itself failed or timed out */
    IMU_ERR_WHOAMI    /* chip responded, but not with 0x47 */
} IMU_Status;

/* Straight out of the sensor, no scaling applied. Index order is the chip's
   own X, Y, Z - not the flight-controller frame. */
typedef struct
{
    int16_t gyro[3];
    int16_t accel[3];
    int16_t temp;
} IMU_Raw_t;

typedef struct
{
    float gyro[3];    /* degrees per second, chip axes */
    float accel[3];   /* g, chip axes */
    float temp_c;
} IMU_Data_t;

/* Body frame used by everything downstream: X forward, Y right, Z down.
   Bias-corrected and low-pass filtered. */
typedef struct
{
    float roll;    /* deg/s, positive = right side down */
    float pitch;   /* deg/s, positive = nose up */
    float yaw;     /* deg/s, positive = nose right */
} IMU_Rates_t;

IMU_Status IMU_Init(SPI_HandleTypeDef *hspi);

/* Averages the gyro at rest to find its zero offset. The craft must be
   completely still. Returns false if it detected movement. */
bool IMU_Calibrate(uint16_t samples);

/* Reads one sample, applies bias correction, axis transform and filtering. */
bool IMU_Update(void);

IMU_Rates_t IMU_GetRates(void);      /* filtered, in the body frame */
IMU_Data_t  IMU_GetData(void);       /* scaled, chip axes, unfiltered */

/* Accelerometer in the body frame (X forward, Y right, Z down), in g. */
void IMU_GetAccelBody(float out[3]);

/* True once per new sample; consumes the flag. Driven by the INT1 pin. */
bool     IMU_DrdyPending(void);
uint32_t IMU_DrdyCount(void);

/* Samples that arrived before the loop consumed the previous one. Any
   sustained growth here means the loop is not keeping up. */
uint32_t IMU_DrdyMissed(void);
void     IMU_DrdyResetStats(void);

/* Enable slow bias re-measurement. Only ever true while disarmed. */
void IMU_AllowBiasTracking(bool allow);
void IMU_GetBias(float out[3]);

bool IMU_ReadRaw(IMU_Raw_t *out);
void IMU_Convert(const IMU_Raw_t *raw, IMU_Data_t *out);

bool    IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t IMU_ReadReg(uint8_t reg);
bool    IMU_WriteReg(uint8_t reg, uint8_t value);
uint8_t IMU_WhoAmI(void);

#endif /* IMU_H */
