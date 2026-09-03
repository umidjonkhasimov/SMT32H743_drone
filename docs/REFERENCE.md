# Reference

## Tunable constants

All of these live in the `USER CODE BEGIN PD` block of `Core/Src/main.c`
unless noted.

### Channel map

| Constant | Value | Meaning |
|---|---|---|
| `CH_ROLL` | 0 | Channel 1 |
| `CH_PITCH` | 1 | Channel 2 - **negated in code**, this transmitter inverts it |
| `CH_THROTTLE` | 2 | Channel 3 |
| `CH_YAW` | 3 | Channel 4 |
| `CH_ARM` | 4 | Channel 5, switch high = armed |
| `CH_MODE` | 5 | Channel 6, switch high = angle mode |

### Rates and angles

| Constant | Value | Effect of raising it |
|---|---|---|
| `MAX_RATE_ROLL` | 400 deg/s | Faster rolls at full stick, twitchier everywhere |
| `MAX_RATE_PITCH` | 400 deg/s | Same, pitch axis |
| `MAX_RATE_YAW` | 300 deg/s | Faster yaw |
| `MAX_ANGLE_DEG` | 30 deg | Steeper maximum lean in angle mode |
| `ANGLE_P` | 6.0 /s | Snappier return to level; too high oscillates around level |

Competition acro setups run rates closer to 800 deg/s. These are deliberately
gentle.

### Throttle and arming

| Constant | Value | Meaning |
|---|---|---|
| `DSHOT_THROTTLE_MIN` | 48 | DShot 0-47 are reserved commands, not throttle |
| `DSHOT_THROTTLE_SPAN` | 1999 | 48 + 1999 = 2047, the maximum |
| `ARM_THROTTLE_MAX` | 0.05 | Throttle must be below this to arm |
| `IDLE_THROTTLE` | 0.05 | Below this the PID is held in reset |
| `IDLE_MOTOR` | 0.04 | Minimum spin while armed |

### Filtering and estimation

| Constant | Where | Value | Notes |
|---|---|---|---|
| `IMU_GYRO_LPF_HZ` | `imu.h` | 100 Hz | Gyro noise filter |
| PID D cutoff | `main.c`, `PID_Init` arg | 60 Hz | Lower if motors run hot or buzz |
| `ACCEL_LPF_HZ` | `attitude.c` | 10 Hz | Must stay well below prop frequency |
| `ATTITUDE_TAU` | `main.c` | 1.0 s | Higher trusts the gyro longer |
| `ACCEL_TRUST_LO/HI` | `attitude.c` | 0.80 / 1.20 g | Band where gravity is believed |
| `BIAS_TRACK_MAX_RATE` | `imu.c` | 2.0 deg/s | Above this, assumed to be moving |
| `BIAS_TRACK_GAIN` | `imu.c` | 0.0001 | ~10 s to absorb 1 deg/s |
| `MIXER_MAX_BOOST` | `mixer.c` | 0.10 | Most the mixer may raise throttle |

### PID gains

Set in `main.c` via `PID_Init(pid, kp, ki, kd, iLimit, dCutoffHz, dt)`:

| Axis | Kp | Ki | Kd | iLimit |
|---|---|---|---|---|
| Roll | 0.0015 | 0.0020 | 0.000020 | 0.10 |
| Pitch | 0.0015 | 0.0020 | 0.000020 | 0.10 |
| Yaw | 0.0025 | 0.0040 | 0 | 0.10 |

Yaw has no D term. Yaw responds slowly and is dominated by motor spin-up
rather than aerodynamics, so a derivative mostly amplifies noise.

`g_pidRoll`, `g_pidPitch` and `g_pidYaw` are globals specifically so gains
can be changed live in the debugger without reflashing.

## Diagnostics

Every one of these is a global in `main.c`, readable with `p <name>` in GDB.

### Startup health

| Variable | Good value | Meaning |
|---|---|---|
| `g_imu_id` | `0x47` | IMU WHO_AM_I. `0x00` = MISO wrong, `0xFF` = not connected |
| `g_imu_status` | `IMU_OK` | Result of IMU init |
| `g_imu_attempts` | 1 | Init tries needed; >1 means the rail was still settling |
| `g_cal_ok` | `true` | False means it moved during calibration, or no samples arrived |

### Loop timing

| Variable | Good value | Meaning |
|---|---|---|
| `g_loop_count` | climbing | **If 0, the loop never ran** - the gyro interrupt is not firing |
| `g_dt` | ~0.001 | Seconds between samples |
| `g_loop_us_min` | ~1000 | Shortest interval seen, microseconds |
| `g_loop_us_max` | ~1000 | Longest. Large values mean something stalls the loop |
| `g_drdy_missed` | 0 | Samples that arrived before the loop consumed the previous one |

`g_loop_count` is the single most useful variable when nothing works: it
distinguishes "the firmware is dead" from "the firmware is running but
something downstream is wrong."

### Sensors

| Variable | Meaning |
|---|---|
| `g_rates` | Filtered body-frame rates, deg/s - roll, pitch, yaw |
| `g_imu_data` | Scaled chip-axis gyro/accel/temperature, unfiltered |
| `g_imu_raw` | Raw 16-bit counts |
| `g_gyroBias` | Current gyro zero offsets, deg/s |
| `g_attitude` | Estimated roll and pitch angles, degrees |
| `g_accelTrusted` | Whether this sample's accelerometer was believed |
| `g_trustRatio` | Fraction believed since boot. **Should exceed 0.8 in flight** |

### Control

| Variable | Meaning |
|---|---|
| `g_armed` | Arming latch state |
| `g_angleMode` | True when the mode switch is high |
| `g_setpoint[3]` | Demanded rates, deg/s |
| `g_pidOut[3]` | Controller output, mix-authority units |
| `g_motorMix[4]` | Final motor demands, 0..1, in M1..M4 order |
| `g_mixClipped` | Fraction of correction the mixer could not deliver |
| `motors[i].dropped` | DShot frames skipped because the previous had not finished |

### Faults

`g_fault` lives in no-init RAM and **survives a reset but not a power
cycle**. A stale record from an earlier fault will still be there, so clear
it by fully disconnecting power rather than pressing RST.

| Field | Meaning |
|---|---|
| `magic` | `0xFA017ED` means a fault genuinely occurred; anything else means nothing did |
| `source` | 1 = HardFault, 2 = MemManage, 3 = BusFault, 4 = UsageFault |
| `cfsr` | The main one. Bit 10 = imprecise bus error, bit 16 = undefined instruction, bit 17 = invalid state |
| `hfsr` | Bit 30 = escalated from a more specific fault, so read `cfsr` |
| `bfar` | Faulting address, only valid if CFSR bit 15 is set |
| `abfsr` | Cortex-M7 auxiliary bus fault. Bit 3 = AXIM, bits [9:8] = 2 means SLVERR |
| `icsr` | Bits [8:0] = active exception number |
| `stack[20]` | Raw stack. The 8-word exception frame is in here |

Finding the fault location: the exception frame is `R0 R1 R2 R3 R12 LR PC
xPSR`. `xPSR` is recognisable because bit 24 is set, and the word before it
is the PC. Resolve it with:

```
arm-none-eabi-addr2line -f -e build/Debug/SMT32H743_drone.elf 0x<pc>
```

The exception number in that stacked `xPSR` (bits [8:0]) tells you which
interrupt was running when it faulted. Subtract 16 for the IRQ number.

### CRSF link

Read with `crsf_stats()` and `crsf_link()`, or inspect the statics in
`rx.c` directly.

| Field | Good value | Meaning |
|---|---|---|
| `frames_good` | climbing ~250/s | Valid frames received |
| `frames_crc_err` | near 0 | Corrupt frames. A handful at link-up is normal |
| `resyncs` | 0 | Parser discarded a partial frame |
| `uart_errors` | near 0 | Overrun, framing or noise errors |

A healthy link measured on this hardware: 1 CRC error in ~8700 frames.

## Debugging notes

**Do not single-step through the main loop.** With four DMA interrupts and
CRSF traffic every millisecond, a step almost always lands inside an
interrupt handler, which GDB reports as `SIGTRAP`. That means "the debugger
stopped the CPU", not "the program crashed". Use breakpoints and `continue`,
or read variables while it runs.

**Put breakpoints after the arming loop**, never inside it. ESCs need
uninterrupted DShot to arm.

**Timers freeze when the debugger halts** (`__HAL_DBGMCU_FREEZE_TIM1/5`), so
DShot frames end cleanly at a boundary rather than being chopped mid-pulse.
The ESCs will time out and beep after a few seconds of that, which is
correct behaviour.

**To debug on battery power**, connect the ST-Link's SWD and GND pins only,
not its power pin.
