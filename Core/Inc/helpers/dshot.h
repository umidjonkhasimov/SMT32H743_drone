#ifndef DSHOT_H
#define DSHOT_H

#include "main.h"

#define DSHOT_FRAME_SIZE       16
#define DSHOT_RESET_SLOTS      2
#define DSHOT_DMA_BUFFER_SIZE  (DSHOT_FRAME_SIZE + DSHOT_RESET_SLOTS)

typedef enum
{
    DSHOT150 = 150000,
    DSHOT300 = 300000,
    DSHOT600 = 600000,
} DShotSpeed;

typedef struct
{
    uint16_t psc;
    uint16_t arr;
    uint16_t t0h;
    uint16_t t1h;
} DShotTiming;

DShotTiming DShot_GetTiming(
    DShotSpeed speed,
    uint32_t timerClock
);

uint16_t DShot_MakePacket(
    uint16_t throttle,
    uint8_t telemetry
);

void DShot_EncodeBuffer(
    uint16_t packet,
    uint32_t *buffer,
    const DShotTiming *timing
);

#endif