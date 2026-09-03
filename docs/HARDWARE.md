# Hardware

## Board

**LXB743ZI-P1** (Lu Xiaoban), carrying an **STM32H743ZIT6** in LQFP144.
This is not an ST Nucleo - do not assume Nucleo header positions or onboard
peripherals apply.

## Pinout

Everything the firmware touches:

| Pin | Function | Peripheral | Notes |
|---|---|---|---|
| PE9 | Motor 1 (rear-right) | TIM1_CH1, DMA1 Stream 0 | DShot600 |
| PE11 | Motor 2 (front-right) | TIM1_CH2, DMA1 Stream 1 | DShot600 |
| PA0 | Motor 3 (rear-left) | TIM5_CH1, DMA1 Stream 2 | DShot600 |
| PA1 | Motor 4 (front-left) | TIM5_CH2, DMA1 Stream 3 | DShot600 |
| PC6 | USART6_TX | USART6 | Unused - RX-only mode |
| PC7 | USART6_RX | USART6, DMA1 Stream 4 | CRSF in, 420000 baud |
| PA4 | IMU chip select | GPIO output | Plain GPIO, **not** hardware NSS |
| PA5 | SPI1_SCK | SPI1 | 8 MHz |
| PA6 | SPI1_MISO | SPI1 | |
| PA7 | SPI1_MOSI | SPI1 | |
| PE10 | IMU INT1 (data ready) | EXTI10 | Rising edge |

Chip select is driven manually rather than by the SPI peripheral. Hardware
NSS timing is fiddly and misbehaves once a second device shares the bus;
setting the pin by hand is one line each way and always does what you expect.

## Motor layout

Quad-X, viewed from above, nose pointing up:

```
   M4   M2          M1 = rear-right   (CW)
     \ /            M2 = front-right  (CCW)
     / \            M3 = rear-left    (CCW)
   M3   M1          M4 = front-left   (CW)
```

Diagonal pairs spin the same direction, which is what makes yaw control
possible. This matches the Betaflight default ordering.

`motors[0]`..`motors[3]` in the code correspond to M1..M4.

## IMU

**ICM-42688-P** on a breakout module, connected over SPI1.

Mounted with its **X axis pointing right, Y towards the nose, Z up**.
The firmware works in the standard flight-control body frame - **X forward,
Y right, Z down** - so `imu.c` converts:

```
roll  = +gyro_Y      (chip Y is forward)
pitch = +gyro_X      (chip X is right)
yaw   = -gyro_Z      (chip Z is up, body Z is down)
```

Both frames are right-handed. Swapping X and Y flips the handedness and the
Z negation flips it back; dropping that negation silently inverts yaw.

Verified on hardware: rolling right-side-down gives positive roll, nose-up
gives positive pitch, nose-right gives positive yaw, and `accel[2]` reads
about +1.0 g sitting level.

Configuration: gyro +-2000 deg/s, accel +-16 g, both at 1 kHz, low-noise
mode. The wide ranges are deliberate - a quad in an acro flip exceeds
1000 deg/s, and prop transients spike the accelerometer well past 1 g. A
sensor that saturates mid-manoeuvre blinds the controller exactly when it
matters.

## Receiver

CRSF (ExpressLRS) at 420000 baud into **PC7**. The receiver's *TX* pad goes
to PC7, which is the STM32's RX. USART6 is configured receive-only, so
telemetry back to the handset is not supported.

Default AETR channel order:

| Channel | Index in code | Function |
|---|---|---|
| 1 | `CH_ROLL` (0) | Roll |
| 2 | `CH_PITCH` (1) | Pitch - **inverted** by this transmitter |
| 3 | `CH_THROTTLE` (2) | Throttle |
| 4 | `CH_YAW` (3) | Yaw |
| 5 | `CH_ARM` (4) | Arm switch |
| 6 | `CH_MODE` (5) | High = angle mode, low = acro |

## Clock tree

| Domain | Frequency | Source |
|---|---|---|
| CPU | 480 MHz | HSI 64 MHz -> PLL1P (M=4, N=60, P=2) |
| HCLK / AXI | 240 MHz | HPRE = /2 |
| APB1 / APB2 | 120 MHz | /2, so timer clock is 240 MHz |
| SPI1 kernel | 64 MHz | **PER_CK** (from HSI) |
| USART6 | 120 MHz | D2PCLK2 |

**There is no HSE crystal** - the PLL runs from the internal HSI oscillator.
This is fine here. HSI is roughly +-1%, and a UART tolerates about +-2%;
measured CRSF error rate is 1 bad frame in ~8700. SPI does not care at all,
because the clock travels down the wire with the data.

**SPI1/2/3 cannot use PLL1Q** on this part: PLL1Q is 480 MHz and the SPI
kernel input is capped at 200 MHz. PER_CK at 64 MHz sidesteps that without
enabling another PLL. With prescaler 8 that gives an 8 MHz SPI clock, well
under the ICM-42688-P's 24 MHz limit - deliberately conservative for
jumper-wire signal integrity.

## Memory

| Region | Base | Used for |
|---|---|---|
| DTCM | 0x20000000 | Stack, ordinary variables. **Not DMA-accessible.** |
| AXI SRAM | 0x24000000 | `.dma_buffer` section - all DMA buffers |

DMA1 lives in the D2 domain and **cannot reach DTCM**, so every buffer a DMA
touches must be placed in `.dma_buffer`, which the linker maps to AXI SRAM:

- `s_rx_buf` (CRSF ring, 256 B) at 0x24000000
- `g_fault` (fault record, 120 B) at 0x24000100
- `motors[4]` (DShot frames, 512 B) at 0x24000180

`.dma_buffer` is declared `NOLOAD`, so startup code never zeroes it. That is
what lets the fault record survive a reset - though not a power cycle.

## Caches

**I-cache on, D-cache off.** Code runs from flash, which is far slower than
a 480 MHz core, so the instruction cache is most of the available
performance and carries no coherency risk.

The data cache is deliberately disabled. It would require every DMA buffer
to be cleaned and invalidated correctly, for very little gain on data this
small. Both cache maintenance call sites are guarded on
`SCB->CCR & SCB_CCR_DC_Msk`, so they do nothing today and start working
automatically if the D-cache is ever enabled.

That guard is not cosmetic: cache maintenance operations are **not** free
no-ops when the cache is off. They still issue bus transactions, and on this
part an unguarded `SCB_InvalidateDCache_by_Addr` returned an AXI slave error
and faulted. See [ARCHITECTURE.md](ARCHITECTURE.md#fault-capture).

## Power

The STM32 and ESCs both run from the flight battery. Startup waits 300 ms
before touching any peripheral, and IMU init retries up to five times, since
on battery the MCU begins executing while its rail is still ramping and the
ESCs are drawing inrush current.
