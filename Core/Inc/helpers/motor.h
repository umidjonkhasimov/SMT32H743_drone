#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "dshot.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    DShotTiming timing;
    uint32_t dropped;   /* frames skipped because the previous one had not finished */
    __attribute__((aligned(32)))
    uint32_t dmaBuffer[DSHOT_DMA_BUFFER_SIZE];
} Motor_t;

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel, DShotSpeed speed, uint32_t timerClock);
void Motor_Write(Motor_t *motor, uint16_t throttle);

#endif