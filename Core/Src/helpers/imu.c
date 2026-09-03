#include "helpers/imu.h"
#include "helpers/filter.h"
#include <string.h>

/* ---- ICM-42688-P register map, user bank 0 ---- */
#define REG_DEVICE_CONFIG 0x11u
#define REG_INT_CONFIG    0x14u
#define REG_TEMP_DATA1    0x1Du   /* first of 14 contiguous data bytes */
#define REG_INT_STATUS    0x2Du
#define REG_PWR_MGMT0     0x4Eu
#define REG_GYRO_CONFIG0  0x4Fu
#define REG_ACCEL_CONFIG0 0x50u
#define REG_INT_CONFIG1   0x64u
#define REG_INT_SOURCE0   0x65u
#define REG_WHO_AM_I      0x75u
#define REG_BANK_SEL      0x76u

/* Full-scale selects, shifted into bits [7:5] of the CONFIG0 registers. */
#define GYRO_FS_2000DPS   0x00u
#define ACCEL_FS_16G      0x00u

/* Output data rate, bits [3:0] of the same registers. */
#define ODR_1KHZ          0x06u
#define ODR_8KHZ          0x03u

/* Scaling for the full-scale ranges selected above. 32768 counts spans the
   full range in each direction, so 32768/2000 = 16.384 counts per deg/s. */
#define GYRO_LSB_PER_DPS  16.384f
#define ACCEL_LSB_PER_G   2048.0f

/* The first byte of every transaction is the register address. Its top bit
   selects direction: 1 = read, 0 = write. */
#define IMU_READ_BIT      0x80u

/* A 16-byte burst at 8 MHz takes ~17 us, so this is enormously generous -
   it only ever trips if the bus is dead. */
#define IMU_SPI_TIMEOUT   10u   /* ms */

static SPI_HandleTypeDef *s_hspi;
static volatile uint32_t  s_drdy_count;
static volatile bool      s_drdy_flag;
static volatile uint32_t  s_drdy_missed;

/* Gyro zero offset in deg/s, measured by IMU_Calibrate. */
static float       s_bias[3];
static PT1_t       s_gyro_lpf[3];
static IMU_Data_t  s_data;
static float       s_accelBody[3];
static IMU_Rates_t s_rates;

/* Raw counts of spread tolerated across a calibration run. 200 counts at
   16.384 LSB per deg/s is about 12 deg/s of movement. */
#define IMU_CAL_MAX_SPREAD 200

/* Slow background bias tracking, used only while disarmed and still.
   A gyro's zero point moves as it warms, and a stale offset is invisible to
   the controller: if the bias is -0.5 deg/s and the craft rotates at exactly
   0.5 deg/s, the gyro reads zero, the PID sees no error, and nothing
   corrects it. Re-measuring between flights keeps that from building up. */
#define BIAS_TRACK_MAX_RATE 2.0f      /* deg/s - above this it is moving */
#define BIAS_TRACK_GAIN     0.0001f   /* ~10 s to absorb a 1 deg/s offset */

static bool s_biasTracking;

static inline void imu_select(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static inline void imu_deselect(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

/* One continuous transaction: address byte out, then len bytes clocked in
   while we send zeros. TransmitReceive rather than Transmit-then-Receive
   because the H7 SPI wants a single uninterrupted session per CS assertion. */
bool IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx[1u + IMU_MAX_BURST] = {0};
    uint8_t rx[1u + IMU_MAX_BURST] = {0};

    if (s_hspi == NULL || len == 0u || len > IMU_MAX_BURST)
    {
        return false;
    }

    tx[0] = reg | IMU_READ_BIT;

    imu_select();
    const HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(s_hspi, tx, rx, (uint16_t)(len + 1u),
                                IMU_SPI_TIMEOUT);
    imu_deselect();

    if (st != HAL_OK)
    {
        return false;
    }

    /* rx[0] was clocked out while we were still sending the address, so the
       register contents start one byte in. */
    memcpy(buf, &rx[1], len);

    return true;
}

uint8_t IMU_ReadReg(uint8_t reg)
{
    uint8_t v = 0;

    (void)IMU_ReadRegs(reg, &v, 1u);

    return v;
}

bool IMU_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };

    if (s_hspi == NULL)
    {
        return false;
    }

    imu_select();
    const HAL_StatusTypeDef st =
        HAL_SPI_Transmit(s_hspi, tx, 2u, IMU_SPI_TIMEOUT);
    imu_deselect();

    return (st == HAL_OK);
}

uint8_t IMU_WhoAmI(void)
{
    return IMU_ReadReg(REG_WHO_AM_I);
}

/* Temperature, accel and gyro sit in 14 contiguous registers, so one burst
   gets a coherent set - all from the same sample, which separate reads
   would not guarantee. Each value is big-endian: high byte first. */
bool IMU_ReadRaw(IMU_Raw_t *out)
{
    uint8_t b[14];

    if (out == NULL || !IMU_ReadRegs(REG_TEMP_DATA1, b, sizeof(b)))
    {
        return false;
    }

    out->temp     = (int16_t)((uint16_t)b[0]  << 8 | b[1]);
    out->accel[0] = (int16_t)((uint16_t)b[2]  << 8 | b[3]);
    out->accel[1] = (int16_t)((uint16_t)b[4]  << 8 | b[5]);
    out->accel[2] = (int16_t)((uint16_t)b[6]  << 8 | b[7]);
    out->gyro[0]  = (int16_t)((uint16_t)b[8]  << 8 | b[9]);
    out->gyro[1]  = (int16_t)((uint16_t)b[10] << 8 | b[11]);
    out->gyro[2]  = (int16_t)((uint16_t)b[12] << 8 | b[13]);

    return true;
}

void IMU_Convert(const IMU_Raw_t *raw, IMU_Data_t *out)
{
    for (int i = 0; i < 3; i++)
    {
        out->gyro[i]  = (float)raw->gyro[i]  / GYRO_LSB_PER_DPS;
        out->accel[i] = (float)raw->accel[i] / ACCEL_LSB_PER_G;
    }

    out->temp_c = ((float)raw->temp / 132.48f) + 25.0f;
}

IMU_Status IMU_Init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;

    /* CS idles high. The chip powers up listening for I2C and latches into
       SPI mode on the first falling edge of CS, so getting this order right
       matters. */
    imu_deselect();
    HAL_Delay(1);

    uint8_t id = 0;

    if (!IMU_ReadRegs(REG_WHO_AM_I, &id, 1u))
    {
        return IMU_ERR_SPI;
    }

    if (id != ICM42688_WHOAMI)
    {
        return IMU_ERR_WHOAMI;
    }

    IMU_WriteReg(REG_BANK_SEL, 0x00u);

    /* Soft reset, then let the chip come back up. */
    IMU_WriteReg(REG_DEVICE_CONFIG, 0x01u);
    HAL_Delay(2);

    IMU_WriteReg(REG_BANK_SEL, 0x00u);
    (void)IMU_ReadReg(REG_INT_STATUS);      /* clears the reset-done flag */

    /* Gyro +-2000 dps, accel +-16 g, both at 1 kHz. Wide ranges because a
       quadcopter sees violent rotation and prop transients. */
    IMU_WriteReg(REG_GYRO_CONFIG0,  (uint8_t)((GYRO_FS_2000DPS << 5) | ODR_1KHZ));
    IMU_WriteReg(REG_ACCEL_CONFIG0, (uint8_t)((ACCEL_FS_16G   << 5) | ODR_1KHZ));

    /* INT1 active high, push-pull, pulsed - matching the rising-edge EXTI
       we configured on PE10. */
    IMU_WriteReg(REG_INT_CONFIG, 0x03u);

    /* INT_ASYNC_RESET powers up as 1, but the datasheet requires it cleared
       for the INT pins to behave correctly. */
    uint8_t cfg1 = IMU_ReadReg(REG_INT_CONFIG1);
    cfg1 &= (uint8_t)~(1u << 4);
    IMU_WriteReg(REG_INT_CONFIG1, cfg1);

    /* Route data-ready to INT1. */
    IMU_WriteReg(REG_INT_SOURCE0, (uint8_t)(1u << 3));

    /* Power up gyro and accel in low-noise mode. */
    IMU_WriteReg(REG_PWR_MGMT0, 0x0Fu);

    /* The gyro needs tens of milliseconds to stabilise before its output is
       trustworthy. */
    HAL_Delay(100);

    /* Until IMU_Calibrate runs these pass the signal straight through with
       no bias correction, so IMU_Update is still safe to call. */
    for (int i = 0; i < 3; i++)
    {
        s_bias[i] = 0.0f;
        PT1_Init(&s_gyro_lpf[i], IMU_GYRO_LPF_HZ, 1.0f / IMU_SAMPLE_RATE_HZ);
    }

    s_drdy_flag = false;

    return IMU_OK;
}

/* ---------------------------------------------------------------------------
   Sample processing
   ------------------------------------------------------------------------ */

/* Board orientation. This IMU is mounted with its X axis pointing right, Y
   towards the nose and Z up. The rest of the firmware works in the standard
   flight-control body frame: X forward, Y right, Z down.

   Both frames are right-handed, so the conversion is a swap of X and Y plus
   a negation of Z - the negation is what keeps it right-handed, and dropping
   it would silently invert yaw. */
static IMU_Rates_t body_from_chip(const float g[3])
{
    IMU_Rates_t r;

    r.roll  =  g[1];   /* chip Y points forward -> roll axis */
    r.pitch =  g[0];   /* chip X points right   -> pitch axis */
    r.yaw   = -g[2];   /* chip Z points up, body Z points down */

    return r;
}

bool IMU_Calibrate(uint16_t samples)
{
    int32_t  sum[3] = {0, 0, 0};
    int16_t  lo[3], hi[3];
    IMU_Raw_t raw;
    uint16_t got = 0;

    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;

    for (int i = 0; i < 3; i++)
    {
        lo[i] = INT16_MAX;
        hi[i] = INT16_MIN;
    }

    for (uint16_t n = 0; n < samples; n++)
    {
        /* Wait for a fresh sample rather than re-reading the same one. The
           guard stops us hanging forever if INT1 is not connected. */
        uint32_t guard = 0;

        while (!IMU_DrdyPending())
        {
            if (++guard > 2000000u)
            {
                return false;
            }
        }

        if (!IMU_ReadRaw(&raw))
        {
            continue;
        }

        for (int i = 0; i < 3; i++)
        {
            sum[i] += raw.gyro[i];

            if (raw.gyro[i] < lo[i]) { lo[i] = raw.gyro[i]; }
            if (raw.gyro[i] > hi[i]) { hi[i] = raw.gyro[i]; }
        }

        got++;
    }

    if (got == 0u)
    {
        return false;
    }

    /* If any axis moved more than this during the average, the craft was not
       still and the offset we just computed is worthless. */
    for (int i = 0; i < 3; i++)
    {
        if ((int32_t)hi[i] - (int32_t)lo[i] > IMU_CAL_MAX_SPREAD)
        {
            return false;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        s_bias[i] = (float)sum[i] / (float)got / GYRO_LSB_PER_DPS;
    }

    for (int i = 0; i < 3; i++)
    {
        PT1_Init(&s_gyro_lpf[i], IMU_GYRO_LPF_HZ, 1.0f / IMU_SAMPLE_RATE_HZ);
    }


    return true;
}

bool IMU_Update(void)
{
    IMU_Raw_t raw;

    if (!IMU_ReadRaw(&raw))
    {
        return false;
    }

    IMU_Convert(&raw, &s_data);

    float g[3];

    for (int i = 0; i < 3; i++)
    {
        /* Bias first, then filter - filtering a biased signal would just
           produce a smooth wrong answer. */
        g[i] = PT1_Apply(&s_gyro_lpf[i], s_data.gyro[i] - s_bias[i]);
    }

    /* With the craft disarmed and sitting still, anything left over after
       bias correction IS bias, so walk the stored offset towards it. The
       gain is deliberately tiny - this must never chase real motion. */
    if (s_biasTracking)
    {
        bool still = true;

        for (int i = 0; i < 3; i++)
        {
            if (g[i] > BIAS_TRACK_MAX_RATE || g[i] < -BIAS_TRACK_MAX_RATE)
            {
                still = false;
            }
        }

        if (still)
        {
            for (int i = 0; i < 3; i++)
            {
                s_bias[i] += g[i] * BIAS_TRACK_GAIN;
            }
        }
    }

    s_rates = body_from_chip(g);

    /* Accelerometer through the same frame conversion, for the attitude
       estimator. Unfiltered: the complementary filter does its own
       smoothing, and filtering twice just adds delay. */
    s_accelBody[0] =  s_data.accel[1];   /* forward */
    s_accelBody[1] =  s_data.accel[0];   /* right   */
    s_accelBody[2] = -s_data.accel[2];   /* down    */

    return true;
}

IMU_Rates_t IMU_GetRates(void) { return s_rates; }
IMU_Data_t  IMU_GetData(void)  { return s_data; }

void IMU_GetAccelBody(float out[3])
{
    out[0] = s_accelBody[0];
    out[1] = s_accelBody[1];
    out[2] = s_accelBody[2];
}

bool IMU_DrdyPending(void)
{
    if (!s_drdy_flag)
    {
        return false;
    }

    s_drdy_flag = false;

    return true;
}

uint32_t IMU_DrdyCount(void) { return s_drdy_count; }

void IMU_AllowBiasTracking(bool allow) { s_biasTracking = allow; }

void IMU_GetBias(float out[3])
{
    out[0] = s_bias[0];
    out[1] = s_bias[1];
    out[2] = s_bias[2];
}
uint32_t IMU_DrdyMissed(void) { return s_drdy_missed; }

/* Clears the pending flag and the miss counter. Call once immediately
   before entering the control loop, so samples produced during startup -
   when nothing was consuming them - do not count as overruns. */
void IMU_DrdyResetStats(void)
{
    s_drdy_flag   = false;
    s_drdy_missed = 0;
}

/* Fires on every INT1 rising edge. HAL calls this from EXTI15_10_IRQHandler. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IMU_INT_Pin)
    {
        s_drdy_count++;

        /* The flag was still set from last time, so the loop had not yet
           consumed the previous sample - we just lost one. */
        if (s_drdy_flag)
        {
            s_drdy_missed++;
        }

        s_drdy_flag = true;
    }
}
