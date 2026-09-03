#include "helpers/dshot.h"

static const uint16_t DSHOT_MSB_MASK = 0x8000;
#define DSHOT_TICKS_PER_BIT 20

static uint8_t dshot_crc(uint16_t packet)
{
    uint16_t csum = 0;
    uint16_t csum_data = packet >> 4;

    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    return csum & 0x0F;
}

DShotTiming DShot_GetTiming(DShotSpeed speed, uint32_t timerClock)
{
    DShotTiming timing;

    timing.psc = 0;
    timing.arr = timerClock / speed - 1;
    timing.t1h = timing.arr * .70; // 15
    timing.t0h = timing.arr * .35; // 7

    return timing;
}

uint16_t DShot_MakePacket(uint16_t throttle, uint8_t telemetry)
{
    throttle &= 0x07FF;

    uint16_t packet = throttle << 5;

    if (telemetry)
    {
        packet |= (1 << 4);
    }

    packet |= dshot_crc(packet);

    return packet;
}

void DShot_EncodeBuffer(
    uint16_t packet,
    uint32_t *buffer,
    const DShotTiming *timing
)
{
    for (int i = 0; i < DSHOT_FRAME_SIZE; i++)
    {
        if (packet & DSHOT_MSB_MASK)
        {
            buffer[i] = timing->t1h;
        }
        else
        {
            buffer[i] = timing->t0h;
        }

        packet <<= 1;
    }

    /* Keep the line LOW after the frame */
    for (int i = DSHOT_FRAME_SIZE;
         i < DSHOT_DMA_BUFFER_SIZE;
         i++)
    {
        buffer[i] = 0;
    }
}