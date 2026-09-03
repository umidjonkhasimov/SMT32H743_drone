/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "helpers/dshot.h"
#include "helpers/motor.h"
#include "helpers/rx.h"
#include "helpers/imu.h"
#include "helpers/filter.h"
#include "helpers/pid.h"
#include "helpers/mixer.h"
#include "helpers/attitude.h"
#include "helpers/fault.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* AETR channel order, as ELRS sends it by default */
#define CH_ROLL      0u
#define CH_PITCH     1u
#define CH_THROTTLE  2u
#define CH_YAW       3u
#define CH_ARM       4u
#define CH_MODE      5u   /* switch high = angle mode, low = acro */

/* DShot 0..47 are reserved commands; 48..2047 is real throttle. */
#define DSHOT_THROTTLE_MIN  48u
#define DSHOT_THROTTLE_SPAN 1999u

/* Throttle must be below this fraction of travel to arm, and to stay idle. */
#define ARM_THROTTLE_MAX    0.05f

/* Loop iterations ignored before timing statistics start accumulating. */
#define LOOP_STATS_SKIP     10u

/* Full stick deflection, in deg/s. Deliberately gentle for a first build -
   a competition acro setup would be closer to 800. */
#define MAX_RATE_ROLL       400.0f
#define MAX_RATE_PITCH      400.0f
#define MAX_RATE_YAW        300.0f

/* Below this throttle the aircraft is not flying, so the controller is held
   in reset rather than integrating against the ground. */
#define IDLE_THROTTLE       0.05f

/* Minimum spin while armed. Stationary motors give the controller no
   authority at all, and an ESC that has stopped takes time to restart. */
#define IDLE_MOTOR          0.04f

/* Angle mode: full stick commands this much lean, and the outer loop turns
   angle error into a rate demand at this gain (1/s). */
#define MAX_ANGLE_DEG       30.0f
#define ANGLE_P             6.0f

/* Complementary filter time constant, seconds. */
#define ATTITUDE_TAU        1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
Motor_t motors[4];

/* Inspect these in the debugger: g_imu_id should read 0x47. */
volatile uint8_t    g_imu_id;
volatile IMU_Status g_imu_status;
volatile uint32_t   g_imu_attempts;

/* Live sensor data, refreshed every pass of the main loop. */
IMU_Raw_t  g_imu_raw;
IMU_Data_t g_imu_data;
uint32_t   g_imu_drdy;

/* Loop timing diagnostics. dt is seconds; the min/max are microseconds. */
IMU_Rates_t g_rates;
float       g_dt;
uint32_t    g_loop_count;
uint32_t    g_loop_us_min = 0xFFFFFFFFu;
uint32_t    g_loop_us_max;
bool        g_cal_ok;
uint32_t    g_drdy_missed;

/* Controller state, exposed so gains can be tweaked live in the debugger. */
PID_t  g_pidRoll, g_pidPitch, g_pidYaw;
float  g_setpoint[3];      /* roll, pitch, yaw - deg/s */
float  g_pidOut[3];
float  g_motorMix[MIXER_MOTORS];
float  g_mixClipped;
bool   g_armed;
bool   g_angleMode;
Attitude_t g_attitude;
bool   g_accelTrusted;
float  g_trustRatio;
float  g_gyroBias[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* The Cortex-M7 debug unit has a free-running cycle counter, far finer
   grained than the 1 ms SysTick. At 480 MHz it wraps every ~9 seconds.

   Always subtract raw cycle counts and convert afterwards. Unsigned
   subtraction survives the rollover only because the counter wraps at
   exactly 2^32; dividing first moves the wrap point to an arbitrary value
   and breaks that guarantee. */
static uint32_t s_cyclesPerUs = 480u;

static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  s_cyclesPerUs = SystemCoreClock / 1000000u;
}

static inline uint32_t cycles(void)
{
  return DWT->CYCCNT;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Instruction cache. Code runs from flash, which is much slower than the
     480 MHz core, so this is most of the performance the M7 has to offer.
     No coherency concerns: nothing writes to instruction memory.
     The data cache is deliberately left off for now - it would need every
     DMA buffer handled carefully, for very little gain on data this small. */
  SCB_EnableICache();

  /* Route memory-management, bus and usage faults to their own handlers.
     They are disabled out of reset, so everything escalates into a single
     HardFault and the specific cause is lost. */
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
              | SCB_SHCSR_BUSFAULTENA_Msk
              | SCB_SHCSR_MEMFAULTENA_Msk;
  /* USER CODE END 1 */
 
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* On battery the MCU starts executing microseconds after its rail begins
     to ramp, while the ESCs are drawing inrush current and pulling it about.
     Powered over USB the supply is already settled, which is why this only
     misbehaves on battery. Give everything a moment before touching any
     peripheral. */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* Stop the motor timers whenever the debugger halts the core. Without
     this the timers and DMA keep running while the CPU is stopped, which
     truncates the DShot frame in flight and leaves the ESCs seeing a
     corrupted pulse stream. */
  __HAL_DBGMCU_FREEZE_TIM1();
  __HAL_DBGMCU_FREEZE_TIM5();

  DWT_Init();

  /* Interrupt priority plan. Lower number = higher priority. CubeMX puts
     everything at 0, which means nothing can preempt anything - fine while
     all the handlers are short, but the gyro sets the pace of the control
     loop and must never wait behind CRSF frame parsing, which is by far the
     longest handler here. */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);   /* gyro data-ready  */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);/* DShot completion */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 2, 0);
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 0);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);/* CRSF receive     */
  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);

  crsf_start(&huart6);

  /* At cold power-on the IMU's own supply is still ramping while the STM32
     is already running, so the first attempt can find a chip that is not
     ready to answer. A warm reset does not have this problem, which is why
     it appears to work only after pressing RST. */
  g_imu_status = IMU_ERR_SPI;

  for (g_imu_attempts = 1u; g_imu_attempts <= 5u; g_imu_attempts++)
  {
    HAL_Delay(50);

    g_imu_status = IMU_Init(&hspi1);

    if (g_imu_status == IMU_OK)
    {
      break;
    }
  }

  g_imu_id = IMU_WhoAmI();

  /* Measure the gyro's zero offset. The craft must be still for this - it
     returns false if it saw movement, and then the bias stays at zero. */
  g_cal_ok = IMU_Calibrate(1000);

  /* Conservative starting gains - these WILL need tuning on the aircraft.
     The D cutoff is deliberately low; D is where gyro noise does its damage. */
  const float dt = 1.0f / IMU_SAMPLE_RATE_HZ;
  PID_Init(&g_pidRoll,  0.0015f, 0.0020f, 0.000020f, 0.10f, 60.0f, dt);
  PID_Init(&g_pidPitch, 0.0015f, 0.0020f, 0.000020f, 0.10f, 60.0f, dt);
  PID_Init(&g_pidYaw,   0.0025f, 0.0040f, 0.0f,      0.10f, 60.0f, dt);

  Attitude_Init(ATTITUDE_TAU, dt);

  Motor_Init(&motors[0], &htim1, TIM_CHANNEL_1, DSHOT600, 240000000);
  Motor_Init(&motors[1], &htim1, TIM_CHANNEL_2, DSHOT600, 240000000);
  Motor_Init(&motors[2], &htim5, TIM_CHANNEL_1, DSHOT600, 240000000);
  Motor_Init(&motors[3], &htim5, TIM_CHANNEL_2, DSHOT600, 240000000);

  /* ESC arming: 2 s of zero-throttle DShot before anything else. */
  for (int i = 0; i < 2000; i++)
  {
    crsf_update();

    Motor_Write(&motors[0], 0);
    Motor_Write(&motors[1], 0);
    Motor_Write(&motors[2], 0);
    Motor_Write(&motors[3], 0);
    HAL_Delay(1);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Startup produced samples nobody was consuming; that is not an overrun.
     Reset the counters and stamp the clock immediately before the loop. */
  IMU_DrdyResetStats();

  bool     armed = false;
  uint32_t lastCyc = cycles();

  while (1)
  {
    /* The gyro sets the pace. Every iteration is exactly one sensor sample,
       so dt is constant by construction rather than by hoping HAL_Delay was
       accurate - which is what the PID derivative term depends on. */
    if (!IMU_DrdyPending())
    {
      continue;
    }

    const uint32_t nowCyc = cycles();
    const uint32_t elapsedCyc = nowCyc - lastCyc;   /* wrap-safe: raw cycles */
    lastCyc = nowCyc;

    const uint32_t elapsed = elapsedCyc / s_cyclesPerUs;

    g_dt = (float)elapsedCyc / (float)SystemCoreClock;
    g_loop_count++;
    g_drdy_missed = IMU_DrdyMissed();

    /* The first interval spans startup, and the second is only a fragment
       of a sample period - the loop starts at a random phase within it.
       Both would poison the statistics. */
    if (g_loop_count > LOOP_STATS_SKIP)
    {
      if (elapsed < g_loop_us_min) { g_loop_us_min = elapsed; }
      if (elapsed > g_loop_us_max) { g_loop_us_max = elapsed; }
    }

    IMU_Update();
    g_rates    = IMU_GetRates();
    g_imu_data = IMU_GetData();
    g_imu_drdy = IMU_DrdyCount();

    float accelBody[3];
    IMU_GetAccelBody(accelBody);
    Attitude_Update(&g_rates, accelBody, g_dt);
    g_attitude     = Attitude_Get();
    g_accelTrusted = Attitude_AccelTrusted();
    g_trustRatio   = Attitude_TrustRatio();
    IMU_GetBias(g_gyroBias);

    crsf_update();

    const float t = crsf_unit(CH_THROTTLE);

    /* Arming latch. Entering the armed state requires the throttle to be
       down; without that, flipping the arm switch with the stick already up
       spins every motor to that throttle instantly. Losing the link or the
       arm switch drops straight back out. */
    if (crsf_is_failsafe() || !crsf_switch_high(CH_ARM))
    {
      armed = false;
    }
    else if (!armed && t < ARM_THROTTLE_MAX)
    {
      armed = true;
    }

    g_armed = armed;

    /* Re-measure the gyro zero only on the ground. Doing this in flight
       would teach the controller that a real rotation is its own error. */
    IMU_AllowBiasTracking(!armed);

    /* Sticks set the rate we are asking for, not an angle - this is acro
       mode. Centred sticks mean "hold zero rotation", which is what makes
       the aircraft feel locked in rather than self-levelling. */
    const float stickRoll  =  crsf_norm(CH_ROLL);
    const float stickPitch = -crsf_norm(CH_PITCH);   /* TX sends this inverted */
    const float stickYaw   =  crsf_norm(CH_YAW);

    g_angleMode = crsf_switch_high(CH_MODE);

    if (g_angleMode)
    {
      /* Cascaded control: the stick asks for a lean angle, an outer P loop
         turns the angle error into a rate demand, and the same rate PID as
         acro mode flies it. Centre the sticks and the demanded angle is
         zero, so the aircraft levels itself. */
      const float wantRoll  = stickRoll  * MAX_ANGLE_DEG;
      const float wantPitch = stickPitch * MAX_ANGLE_DEG;

      g_setpoint[0] = (wantRoll  - g_attitude.roll)  * ANGLE_P;
      g_setpoint[1] = (wantPitch - g_attitude.pitch) * ANGLE_P;

      /* The outer loop must never ask for more than the airframe can give. */
      if (g_setpoint[0] >  MAX_RATE_ROLL)  { g_setpoint[0] =  MAX_RATE_ROLL; }
      if (g_setpoint[0] < -MAX_RATE_ROLL)  { g_setpoint[0] = -MAX_RATE_ROLL; }
      if (g_setpoint[1] >  MAX_RATE_PITCH) { g_setpoint[1] =  MAX_RATE_PITCH; }
      if (g_setpoint[1] < -MAX_RATE_PITCH) { g_setpoint[1] = -MAX_RATE_PITCH; }
    }
    else
    {
      g_setpoint[0] = stickRoll  * MAX_RATE_ROLL;
      g_setpoint[1] = stickPitch * MAX_RATE_PITCH;
    }

    /* Yaw is always rate-controlled: gravity says nothing about heading. */
    g_setpoint[2] = stickYaw * MAX_RATE_YAW;

    const bool flying = armed && (t > IDLE_THROTTLE);

    if (flying)
    {
      g_pidOut[0] = PID_Update(&g_pidRoll,  g_setpoint[0], g_rates.roll,  g_dt);
      g_pidOut[1] = PID_Update(&g_pidPitch, g_setpoint[1], g_rates.pitch, g_dt);
      g_pidOut[2] = PID_Update(&g_pidYaw,   g_setpoint[2], g_rates.yaw,   g_dt);
    }
    else
    {
      /* Held in reset on the ground: otherwise the integral term winds up
         against an aircraft that physically cannot respond, and dumps that
         accumulated correction into the motors the moment it lifts off. */
      PID_Reset(&g_pidRoll);
      PID_Reset(&g_pidPitch);
      PID_Reset(&g_pidYaw);

      g_pidOut[0] = g_pidOut[1] = g_pidOut[2] = 0.0f;
    }

    float mixThrottle = 0.0f;

    if (armed)
    {
      mixThrottle = flying ? t : IDLE_MOTOR;
    }

    g_mixClipped = Mixer_Apply(mixThrottle,
                               g_pidOut[0], g_pidOut[1], g_pidOut[2],
                               g_motorMix);

    for (int i = 0; i < MIXER_MOTORS; i++)
    {
      uint16_t v = 0;

      if (armed)
      {
        v = (uint16_t)(DSHOT_THROTTLE_MIN + g_motorMix[i] * DSHOT_THROTTLE_SPAN);
      }

      Motor_Write(&motors[i], v);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
