#include "helpers/motor.h"

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim, uint32_t channel, DShotSpeed speed, uint32_t timerClock)
{
    motor->htim = htim;
    motor->channel = channel;
    motor->timing = DShot_GetTiming(speed, timerClock);
    motor->dropped = 0;

    __HAL_TIM_SET_PRESCALER(motor->htim, motor->timing.psc);
    __HAL_TIM_SET_AUTORELOAD(motor->htim, motor->timing.arr);

    Motor_Write(motor, 0);
}

void Motor_Write(Motor_t *motor, uint16_t throttle)
{
    uint16_t packet = DShot_MakePacket(throttle, 0);
    DShot_EncodeBuffer(packet, motor->dmaBuffer, &motor->timing);
    
    /* Only meaningful with the data cache enabled, and not safe without it:
       maintenance ops still generate bus traffic when the cache is off. */
    if (SCB->CCR & SCB_CCR_DC_Msk)
    {
        SCB_CleanDCache_by_Addr((uint32_t*)motor->dmaBuffer, sizeof(motor->dmaBuffer));
    }

    if (HAL_TIM_PWM_Start_DMA(motor->htim, motor->channel,
                              motor->dmaBuffer, DSHOT_DMA_BUFFER_SIZE) != HAL_OK)
    {
        /* The previous frame never completed - usually because the debugger
           halted the core mid-transfer, since the DMA and timer keep running
           while the CPU is stopped. Tear the channel down so the next update
           starts cleanly instead of failing silently forever. */
        motor->dropped++;
        HAL_TIM_PWM_Stop_DMA(motor->htim, motor->channel);
    }
}
