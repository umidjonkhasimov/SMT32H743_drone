# Portability

## What "board agnostic" can and cannot mean

**It cannot mean one binary that flashes to any STM32.** Different cores,
flash layouts, peripherals and HAL APIs make that impossible for any CubeMX
project, not just this one.

**It can mean** that the flight-control logic has no idea what hardware it is
running on, and that moving to a different MCU means rewriting a small,
clearly-bounded transport layer instead of the whole firmware.

That is the target, and this document is the plan to get there.

## Where the code stands today

Roughly 1540 lines under `Core/*/helpers/`, plus about 200 lines of USER CODE
in `main.c`.

### Tier 1 - already fully portable (~410 lines)

No hardware dependency whatsoever. Would compile on any MCU, or on a PC.

| Module | Lines | Depends on |
|---|---|---|
| `filter.c/h` | 48 | `<stdint.h>` |
| `pid.c/h` | 82 | `filter.h` |
| `mixer.c/h` | 125 | `<stdint.h>` |
| `attitude.c/h` | 155 | `math.h`, `filter.h`, and `IMU_Rates_t` from `imu.h` |

Only one thing spoils this: `attitude.h` includes `imu.h` purely to get the
`IMU_Rates_t` type, which drags in `main.h` and the whole HAL. Moving that
type to a shared header fixes it.

### Tier 2 - portable logic, currently entangled (~225 lines)

Pure algorithms that only touch hardware headers incidentally.

| What | Lines | Entanglement |
|---|---|---|
| `dshot.c` - frame encoding, CRC, bit timing | 74 | Includes `main.h` only for integer types |
| CRSF parsing in `rx.c` - CRC8, channel unpack, frame state machine | ~150 | Sits in the same file as the DMA transport |

### Tier 3 - genuinely hardware-specific (~540 lines)

Must be reimplemented for a different MCU.

| What | Lines | Why |
|---|---|---|
| `motor.c` | 38 | Timer + DMA, cache maintenance |
| CRSF transport in `rx.c` | ~150 | Circular UART DMA, `NDTR`, HAL callbacks |
| SPI transport in `imu.c` | ~120 | `HAL_SPI_TransmitReceive`, GPIO chip select, EXTI |
| `fault.c` | 30 | Cortex-M fault registers; `ABFSR` is M7-only |
| `main.c` USER CODE | ~200 | Loop logic and hardware setup mixed together |

So about **55% of the code is already portable** and the rest is a bounded
rewrite.

## What specifically will not survive a family change

| Thing | Used for | Problem elsewhere |
|---|---|---|
| `Drivers/STM32H7xx_HAL_Driver` | Everything | H7 only. Regenerate in CubeMX for the target |
| `DMA1_Stream0..4` | DShot, CRSF | F0/F1/F3/L0/L4 have DMA **Channels**, not Streams |
| `SCB_EnableICache`, `SCB_CCR_DC_Msk` | Cache control | Cortex-M7 only. M0/M3/M4 have no cache |
| `MPU_Config` | Memory protection | Absent on M0, different on M3/M4 |
| `DWT->CYCCNT` | Microsecond timing | Absent on M0/M0+. Needs a hardware timer instead |
| `.dma_buffer` at `0x24000000` | DMA buffers | H7 memory map. On F4 all SRAM is DMA-reachable |
| DTCM not DMA-accessible | Buffer placement | An H7 gotcha that does not exist on most parts |
| `RCC_SPI123CLKSOURCE_CLKP` | SPI kernel clock | H7-specific clock tree |
| `-mfpu=fpv5-d16 -mfloat-abi=hard` | All the float math | M4 has single-precision only; M0/M3 have no FPU at all |
| `__HAL_DBGMCU_FREEZE_TIM1` | Debugger behaviour | Register names differ by family |

**The FPU one is the most consequential.** This design is float-heavy and
runs a 1 kHz loop. On a Cortex-M0 every operation becomes software emulation
and the design does not fit. Treat "any STM32" as "any STM32 with an FPU" -
realistically F4, F7, H7, G4, L4 and up.

The DShot timing math is already fine: `DShot_GetTiming(speed, timerClock)`
takes the timer clock as a parameter rather than assuming 240 MHz.

## Target structure

```
src/
  flight/                  <- no HAL header may appear anywhere here
    fc_types.h             <- IMU_Rates_t, Attitude_t, shared types
    control.c/h            <- the loop: modes, arming, setpoints
    pid.c/h
    mixer.c/h
    filter.c/h
    attitude.c/h
    dshot_encode.c/h       <- frame building only
    crsf_parse.c/h         <- byte stream in, channels out
  platform/
    platform.h             <- the interface below
    stm32h7/
      plat_time.c          <- DWT
      plat_imu.c           <- ICM-42688-P over SPI1 + EXTI
      plat_motor.c         <- DShot over TIM1/TIM5 + DMA
      plat_rc.c            <- CRSF over USART6 + circular DMA
      plat_fault.c
  config/
    board.h                <- pins, orientation, rates, gains, channel map
```

## The port interface

Everything the flight code needs from any board:

```c
/* Time */
uint32_t plat_micros(void);
void     plat_delay_ms(uint32_t ms);

/* Inertial sensor */
bool plat_imu_init(void);
bool plat_imu_sample_ready(void);                     /* consumes the flag */
bool plat_imu_read(float gyro[3], float accel[3]);    /* body frame: deg/s, g */

/* Motors */
void plat_motor_init(void);
void plat_motor_write(const float demand[4]);         /* 0..1 each, M1..M4 */

/* RC input */
bool  plat_rc_init(void);
void  plat_rc_poll(void);
bool  plat_rc_failsafe(void);
float plat_rc_channel(uint8_t idx);                   /* -1..+1 */
```

Twelve functions. Porting to a new MCU means implementing those and nothing
else.

Note that `plat_imu_read` returns data **already in the body frame**. The
axis transform is board-specific mounting, not flight logic, so it belongs
below the interface.

## Migration plan

Each step is small and independently testable. **Fly-test between steps** -
this is a working aircraft, and a big-bang refactor would leave you unable
to tell which change broke it.

1. **Create `config/board.h`** and move every tunable constant out of
   `main.c`. Pure relocation, no behaviour change.
2. **Create `flight/fc_types.h`** with `IMU_Rates_t` and `Attitude_t`, and
   break `attitude.h`'s dependency on `imu.h`. Tier 1 becomes genuinely
   HAL-free.
3. **Split `rx.c`** into `crsf_parse.c` (portable state machine) and
   `plat_rc.c` (UART + DMA).
4. **Split `imu.c`** into chip register logic and `plat_imu.c` (SPI +
   EXTI), with the axis transform moving below the interface.
5. **Split `motor.c` and `dshot.c`** into `dshot_encode.c` and
   `plat_motor.c`.
6. **Extract the loop** from `main.c` into `flight/control.c`, calling only
   `platform.h`. `main.c` shrinks to CubeMX init plus one call.
7. **Add a build check** that nothing under `flight/` includes a HAL header:

   ```
   grep -rl "stm32h7xx\|main.h" src/flight/ && echo "LEAK" && exit 1
   ```

## The payoff beyond portability

Once `flight/` has no hardware dependency it **compiles on a PC**, which
means the mixer signs, the PID behaviour, the attitude filter and the CRSF
parser can all be unit-tested on a desktop with no aircraft involved.

That matters more than portability here. Two of the worst bugs in this
project so far were an inverted pitch column in the mixer and an attitude
estimator that rejected almost every accelerometer sample. Both were found
by flying. Both would have been caught by a twenty-line test on a laptop.
