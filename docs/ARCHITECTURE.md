# Architecture

## The control loop

Everything hangs off one idea: **the gyro sets the pace.**

```
ICM-42688-P finishes a sample (1 kHz)
        |
        v  INT1 rising edge on PE10
   EXTI15_10_IRQHandler  ->  s_drdy_flag = true
        |
        v
   main loop wakes, and for this one sample:
        |
        +-- IMU_Update()        read 14 bytes over SPI, de-bias, filter,
        |                       convert chip axes -> body frame
        +-- Attitude_Update()   complementary filter -> roll/pitch angles
        +-- crsf_update()       failsafe timer only; parsing is interrupt-driven
        +-- arming latch        link + switch + throttle-down
        +-- setpoints           sticks -> deg/s (acro) or -> angle -> deg/s (angle)
        +-- PID_Update() x3     rate error -> mix authority
        +-- Mixer_Apply()       throttle + roll/pitch/yaw -> 4 motor demands
        +-- Motor_Write() x4    DShot600 frame out over TIM+DMA
```

The loop body is gated on `IMU_DrdyPending()`. There is no `HAL_Delay`, no
timer tick, no software scheduler. Every iteration is exactly one sensor
sample apart, so `dt` is constant *by construction* rather than by hoping a
delay was accurate.

That matters because PID's derivative divides by `dt` and its integral
multiplies by it. Jittery timing produces controller output that moves for
reasons unrelated to how the aircraft is flying, and the D term amplifies
that noise straight onto the motors.

**Consequence worth knowing:** if the gyro stops delivering, the loop body
never runs, motors stop receiving DShot, and the ESCs disarm. Nothing can
stabilise a quad without a gyro, so stopping is the correct failure - but it
presents as "the drone is completely dead" rather than as an error.

## Interrupts

CubeMX puts everything at priority 0, meaning nothing can preempt anything.
That is replaced with a real plan (lower number = higher priority):

| Interrupt | Priority | Rationale |
|---|---|---|
| EXTI15_10 (gyro data-ready) | 1 | Sets the loop pace; must never wait |
| DMA1 Stream 0-3 (DShot done) | 2 | Short, time-sensitive |
| USART6 + DMA1 Stream 4 (CRSF) | 5 | Longest handler - must be preemptible |
| SysTick | 15 | Housekeeping only |

CRSF frame parsing is by far the longest interrupt handler here. Putting it
below the gyro means a sample can interrupt frame parsing rather than
queueing behind it.

## Modules

Under `Core/Src/helpers/` and `Core/Inc/helpers/`.

### `filter.c` - PT1 low-pass (25 lines)

Single-pole low-pass, one multiply and one add per sample. Used for gyro
noise, PID derivative smoothing, and accelerometer conditioning.

```c
f->state += (input - f->state) * f->k;      k = dt / (RC + dt)
```

Pure math. No hardware dependency at all.

### `pid.c` - single-axis rate controller (54 lines)

`PID_Update(p, setpoint, measurement, dt)` returns mix authority, roughly
-1..+1, where 1.0 means the entire throttle range.

Two decisions worth knowing about, both explained in
[CONTROL.md](CONTROL.md): the derivative acts on the **measurement** rather
than the error, and the integral is **clamped**.

### `mixer.c` - quad-X mixer (103 lines)

Turns `(throttle, roll, pitch, yaw)` into four motor demands in 0..1, and
returns how much authority had to be surrendered to keep them in range.

### `attitude.c` - complementary filter (120 lines)

Fuses gyro integration with the accelerometer into absolute roll and pitch.
Angle mode depends entirely on this. No yaw - gravity says nothing about
heading.

### `dshot.c` - DShot protocol (74 lines)

Builds the 16-bit frame - 11 bits throttle, 1 telemetry, 4 CRC - and expands
it into an array of timer compare values, one per bit, plus two zero slots
for the inter-frame gap.

Bit timing follows Betaflight: a "1" holds high for 70% of the bit period, a
"0" for 35%. At DShot600 with a 240 MHz timer clock, ARR is 399.

Pure encoding. No hardware access.

### `motor.c` - DShot transport (38 lines)

Wraps a timer channel plus its DMA stream. `Motor_Write` encodes a frame
into a per-motor buffer in AXI SRAM and starts a DMA transfer to the timer's
compare register.

Checks the HAL return: on `HAL_BUSY` it counts a drop and calls
`HAL_TIM_PWM_Stop_DMA` to clear the wedged channel, so one stalled transfer
does not silently kill that motor for the rest of the flight. Watch
`motors[i].dropped`.

### `rx.c` - CRSF receiver (298 lines)

Circular DMA into a 256-byte ring, drained from the UART idle/half/complete
interrupt. The write head comes from the DMA's `NDTR` register rather than
the `Size` the HAL reports, which makes all three event types behave
identically and stays correct if one is missed.

Frame parsing is a byte-wise state machine that self-syncs on the length and
CRC8 (DVB-S2, poly 0xD5) checks, so garbage costs at most one frame.

Only RC frames (type 0x16) refresh the failsafe timer. Link-statistics
frames (0x14) keep arriving after the transmitter stops sending sticks, so
letting those reset the timer would mean failsafe never fires on a real link
loss.

Channel reads use a critical section, so the main loop can never catch the
ISR halfway through updating the set.

### `imu.c` - ICM-42688-P driver (453 lines)

SPI transport, register map, configuration, calibration, filtering and the
axis transform.

Reads all 14 data bytes in **one burst** so temperature, accelerometer and
gyro come from the same sample. Separate reads would let the chip update
mid-sequence and hand back X from one sample and Z from the next.

Bias handling has two parts: a one-second average at startup with a movement
check, and slow background re-measurement whenever the craft is disarmed and
still, which tracks the gyro's zero point as it warms.

### `fault.c` - fault capture (30 lines)

Called from all four fault handlers. Latches CFSR, HFSR, BFAR, MMFAR, ABFSR,
ICSR, both stack pointers, and 20 words off the top of the stack into
`g_fault`, which lives in no-init RAM and survives a reset.

MemManage, BusFault and UsageFault are enabled explicitly in `main`. Out of
reset they are disabled and everything escalates into a single generic
HardFault, throwing away the classification.

This found a real bug: an imprecise bus fault (CFSR bit 10) with ABFSR
showing AXIM/SLVERR, traced to cache maintenance being performed while the
D-cache was disabled.

## Timing

The DWT cycle counter provides microsecond timing - free-running at the core
clock, far finer than the 1 ms SysTick.

**Always subtract raw cycle counts and convert afterwards.** Unsigned
subtraction survives the 2^32 rollover only because the counter wraps at
exactly 2^32; dividing to microseconds first moves the wrap point to an
arbitrary value and breaks it. That bug produced a garbage interval once
every ~9 seconds.

## What lives in main.c

The USER CODE sections of `main.c` hold the loop itself, all tunable
constants, and the diagnostic globals. Everything outside those markers is
CubeMX-generated and will be overwritten on regeneration.

Two generated-file edits are deliberate and must survive regeneration - both
were made in `SMT32H743_drone.ioc` as well as the generated `.c`, which is
why a regen does not revert them:

- `hdma_usart6_rx.Init.Mode = DMA_CIRCULAR`
- SPI1 data size 8 bits, pin speed VERY_HIGH, kernel clock PER_CK
