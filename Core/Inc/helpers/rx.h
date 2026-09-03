#ifndef RX_H
#define RX_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define CRSF_NUM_CHANNELS   16u
#define CRSF_FAILSAFE_MS    250u   /* no valid frame for this long => failsafe */

/* Raw 11-bit channel range as sent over the air */
#define CRSF_CHANNEL_MIN    172u
#define CRSF_CHANNEL_MID    992u
#define CRSF_CHANNEL_MAX    1811u
#define CRSF_CHANNEL_SPAN   (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)  /* 1639 */

typedef struct
{
    int8_t  rssi_dbm;
    uint8_t lq;         /* link quality, 0..100 % */
    int8_t  snr_db;
    uint8_t rf_mode;
    uint8_t tx_power_idx;
} CrsfLink_t;

/* Diagnostics: watch these while bringing the link up. A healthy ELRS link
   shows frames_good climbing steadily with crc_err and resyncs near zero. */
typedef struct
{
    uint32_t frames_good;
    uint32_t frames_crc_err;
    uint32_t resyncs;       /* parser threw away a partial/garbage frame */
    uint32_t uart_errors;   /* overrun / framing / noise */
} CrsfStats_t;

void crsf_start(UART_HandleTypeDef *huart);
void crsf_update(void);                 /* call from the main loop */

bool     crsf_is_failsafe(void);
uint32_t crsf_age_ms(void);             /* ms since the last valid RC frame */

/* Tear-free snapshot of every channel. out[] must hold CRSF_NUM_CHANNELS. */
void crsf_get_channels(uint16_t *out);

uint16_t crsf_channel(uint8_t idx);     /* raw, 172..1811 */
uint16_t crsf_pulse_us(uint8_t idx);    /* 988..2012 us */
float    crsf_norm(uint8_t idx);        /* -1.0 .. +1.0, centred  (roll/pitch/yaw) */
float    crsf_unit(uint8_t idx);        /* 0.0 .. 1.0             (throttle) */
bool     crsf_switch_high(uint8_t idx); /* >= 75 % of travel */

CrsfLink_t  crsf_link(void);
CrsfStats_t crsf_stats(void);

#endif /* RX_H */
