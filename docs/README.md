# STM32H743 Flight Controller

A quadcopter flight controller written from scratch: DShot600 ESC output,
CRSF receiver input, an ICM-42688-P IMU, and a cascaded rate/angle PID
controller running at 1 kHz.

**Status: flying.** Acro and angle modes both work.

## Documentation map

| Document | Covers |
|---|---|
| [HARDWARE.md](HARDWARE.md) | Board, pinout, wiring, power, clock tree |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Module layout, the control loop, data flow |
| [CONTROL.md](CONTROL.md) | The control theory: PID, mixer, attitude, filters |
| [REFERENCE.md](REFERENCE.md) | Every tunable constant and every diagnostic variable |
| [PORTING.md](PORTING.md) | What is hardware-specific, and how to move to another MCU |

## Quick start

Build:

```
cd build/Debug && cmake --build .
```

Flash:

```
STM32_Programmer_CLI -c port=SWD -w build/Debug/SMT32H743_drone.elf -rst
```

The toolchain is STM32CubeCLT 1.22.0, already on PATH. CMake + Ninja.

## Pre-flight, every time

1. **Props off** for any bench work. Motors spin the instant you arm.
2. Place the craft **still and level** before powering on - the gyro
   calibrates during the first second, and it will not fly straight if it
   was moving.
3. Confirm `g_cal_ok` is true and `g_imu_status` is `IMU_OK`.
4. Arm requires: link alive, arm switch high, **and throttle down**. It
   will refuse to arm with the throttle up, by design.

## What this is not

- No GPS, no altitude hold, no return-to-home.
- No magnetometer, so absolute heading is unknown and yaw drifts slowly in
  acro. See [CONTROL.md](CONTROL.md#yaw-has-no-absolute-reference).
- No blackbox logging yet. Tuning is done by reading globals in the
  debugger - see [REFERENCE.md](REFERENCE.md).
- No bidirectional DShot / RPM filtering.
