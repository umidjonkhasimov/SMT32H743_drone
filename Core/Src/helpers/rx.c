#include "helpers/rx.h"
#include <string.h>

#define CRSF_SYNC_BYTE      0xC8u   /* addressed to the flight controller */
#define CRSF_MAX_FRAME_LEN  62u     /* largest legal value of the 'len' byte */

#define CRSF_TYPE_RC        0x16u
#define CRSF_TYPE_LINKSTATS 0x14u

/* Ring the DMA fills continuously. At 420 kbaud a byte takes ~24 us, so 256
   bytes is ~6 ms of slack - far more than the ~4 ms CRSF frame interval.
   Must be 32-byte aligned and a multiple of 32 so cache maintenance never
   touches a neighbouring object. */
#define CRSF_RX_BUF_SIZE    256u

static UART_HandleTypeDef *s_huart;

__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
static volatile uint8_t s_rx_buf[CRSF_RX_BUF_SIZE];

static volatile uint16_t s_tail;    /* next byte we have not consumed yet */

/* Written by the ISR, read by the main loop under a critical section. */
static volatile uint16_t   s_ch[CRSF_NUM_CHANNELS];
static volatile CrsfLink_t s_link;
static volatile CrsfStats_t s_stats;
static volatile uint32_t   s_last_ms;
static volatile bool       s_failsafe = true;

/* Frame reassembly state - touched only from the ISR. */
static uint8_t s_frame[CRSF_MAX_FRAME_LEN + 2u];
static uint8_t s_idx;

/* ---- CRC8 DVB-S2, poly 0xD5 ---- */
static uint8_t crc8(const uint8_t *d, uint8_t n)
{
    uint8_t crc = 0;

    while (n--)
    {
        crc ^= *d++;

        for (int i = 0; i < 8; i++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0xD5u)
                                : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

/* ---- 22 bytes -> 16 x 11-bit channels, LSB-first packed ---- */
static void unpack_channels(const uint8_t *p)
{
    uint32_t acc = 0;
    unsigned bits = 0, ch = 0;

    for (unsigned i = 0; i < 22u && ch < CRSF_NUM_CHANNELS; i++)
    {
        acc |= (uint32_t)p[i] << bits;
        bits += 8;

        while (bits >= 11u && ch < CRSF_NUM_CHANNELS)
        {
            s_ch[ch++] = (uint16_t)(acc & 0x7FFu);
            acc >>= 11;
            bits -= 11;
        }
    }
}

static void handle_frame(void)
{
    const uint8_t  len = s_frame[1];
    const uint8_t *pl  = &s_frame[3];

    /* CRC covers type + payload; it is the last byte of the frame. */
    if (crc8(&s_frame[2], (uint8_t)(len - 1u)) != s_frame[len + 1u])
    {
        s_stats.frames_crc_err++;
        return;
    }

    s_stats.frames_good++;

    switch (s_frame[2])
    {
    case CRSF_TYPE_RC:
        if (len == 24u)                 /* 1 type + 22 payload + 1 crc */
        {
            unpack_channels(pl);
            /* Only an RC frame refreshes the failsafe timer - link
               statistics keep arriving after the TX stops sending sticks. */
            s_last_ms  = HAL_GetTick();
            s_failsafe = false;
        }
        break;

    case CRSF_TYPE_LINKSTATS:
        if (len == 12u)
        {
            s_link.rssi_dbm     = -(int8_t)pl[0];
            s_link.lq           = pl[2];
            s_link.snr_db       = (int8_t)pl[3];
            s_link.rf_mode      = pl[5];
            s_link.tx_power_idx = pl[6];
        }
        break;

    default:
        break;                          /* ignore everything else */
    }
}

/* Byte-wise state machine. Self-syncs on the len + CRC checks, so garbage
   or a mid-frame start costs at most one frame. */
static void parse_byte(uint8_t b)
{
    if (s_idx == 0u)
    {
        if (b == CRSF_SYNC_BYTE)
        {
            s_frame[s_idx++] = b;
        }
    }
    else if (s_idx == 1u)
    {
        if (b < 2u || b > CRSF_MAX_FRAME_LEN)
        {
            s_stats.resyncs++;
            s_idx = 0;
            return;
        }

        s_frame[s_idx++] = b;
    }
    else
    {
        s_frame[s_idx++] = b;

        if (s_idx >= (uint8_t)(s_frame[1] + 2u))
        {
            handle_frame();
            s_idx = 0;
        }
    }
}

/* Consume everything the DMA has written since we last looked. The write
   head comes straight from NDTR rather than the Size the HAL hands us, so
   this behaves identically for half-transfer, transfer-complete and idle
   events - and stays correct if one of them is ever missed. */
static void crsf_drain(void)
{
    const uint16_t ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(s_huart->hdmarx);
    const uint16_t head = (uint16_t)(CRSF_RX_BUF_SIZE - ndtr);

    if (s_tail == head)
    {
        return;
    }

    /* The DMA wrote this behind the CPU's back, so drop any stale lines.
       Safe to invalidate the whole ring: the CPU only ever reads it.

       Only when the cache is actually on. Maintenance operations are not
       free no-ops with it disabled - they still issue bus transactions,
       and on this part that returns an AXI slave error. */
    if (SCB->CCR & SCB_CCR_DC_Msk)
    {
        SCB_InvalidateDCache_by_Addr(s_rx_buf, (int32_t)sizeof(s_rx_buf));
    }

    while (s_tail != head)
    {
        parse_byte(s_rx_buf[s_tail]);
        s_tail = (uint16_t)((s_tail + 1u) % CRSF_RX_BUF_SIZE);
    }
}

void crsf_start(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    for (unsigned i = 0; i < CRSF_NUM_CHANNELS; i++)
    {
        s_ch[i] = CRSF_CHANNEL_MID;     /* centred, not garbage, before link-up */
    }

    s_tail     = 0;
    s_idx      = 0;
    s_failsafe = true;
    s_last_ms  = HAL_GetTick();
    memset((void *)&s_stats, 0, sizeof(s_stats));
    memset((void *)&s_link, 0, sizeof(s_link));

    /* Circular DMA: started once and never re-armed. */
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, (uint8_t *)s_rx_buf, CRSF_RX_BUF_SIZE);
}

void crsf_update(void)
{
    if (!s_failsafe && (HAL_GetTick() - s_last_ms) > CRSF_FAILSAFE_MS)
    {
        s_failsafe = true;
    }
}

bool     crsf_is_failsafe(void) { return s_failsafe; }
uint32_t crsf_age_ms(void)      { return HAL_GetTick() - s_last_ms; }

void crsf_get_channels(uint16_t *out)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(out, (const void *)s_ch, sizeof(s_ch));
    __set_PRIMASK(primask);
}

uint16_t crsf_channel(uint8_t i)
{
    return (i < CRSF_NUM_CHANNELS) ? s_ch[i] : CRSF_CHANNEL_MID;
}

uint16_t crsf_pulse_us(uint8_t i)
{
    /* Maps 172 -> 988 us and 1811 -> 2012 us. */
    return (uint16_t)(((uint32_t)crsf_channel(i) * 1024u) / 1639u + 881u);
}

float crsf_norm(uint8_t i)
{
    const float v = ((float)crsf_channel(i) - (float)CRSF_CHANNEL_MID)
                  / ((float)CRSF_CHANNEL_SPAN * 0.5f);

    return (v < -1.0f) ? -1.0f : (v > 1.0f) ? 1.0f : v;
}

float crsf_unit(uint8_t i)
{
    const float v = ((float)crsf_channel(i) - (float)CRSF_CHANNEL_MIN)
                  / (float)CRSF_CHANNEL_SPAN;

    return (v < 0.0f) ? 0.0f : (v > 1.0f) ? 1.0f : v;
}

bool crsf_switch_high(uint8_t i) { return crsf_norm(i) >= 0.5f; }

CrsfLink_t crsf_link(void)
{
    CrsfLink_t out;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out = *(const CrsfLink_t *)&s_link;
    __set_PRIMASK(primask);
    return out;
}

CrsfStats_t crsf_stats(void)
{
    CrsfStats_t out;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out = *(const CrsfStats_t *)&s_stats;
    __set_PRIMASK(primask);
    return out;
}

/* ---- HAL hooks ---- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size;   /* the write head comes from NDTR instead */

    if (huart == s_huart)
    {
        crsf_drain();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart)
    {
        return;
    }

    s_stats.uart_errors++;

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                 UART_CLEAR_FEF  | UART_CLEAR_PEF);
    HAL_UART_AbortReceive(huart);

    s_tail = 0;
    s_idx  = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, (uint8_t *)s_rx_buf, CRSF_RX_BUF_SIZE);
}
