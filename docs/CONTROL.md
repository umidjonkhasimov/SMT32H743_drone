# Control

## Body frame

X forward, Y right, Z down. Right-handed.

| Axis | Positive means |
|---|---|
| roll | right side down |
| pitch | nose up |
| yaw | nose right |

Every sign in the firmware follows from this. Getting one wrong does not
produce a slightly-off aircraft - it produces a controller that reinforces
disturbances instead of opposing them, which diverges into a flip.

## Acro mode - rate control

The stick commands a **rotation rate**, not an angle.

```
setpoint = stick * MAX_RATE        (400 deg/s roll/pitch, 300 yaw)
error    = setpoint - measured_rate
output   = PID(error)
```

Centre the sticks and the setpoint is zero, meaning "hold zero rotation" -
which is what makes acro feel locked in. But the aircraft has no idea which
way is up, so it holds whatever attitude it is left in. Hold a stick over
and it keeps rolling, past inverted, until you centre it.

## Angle mode - cascaded control

Two nested loops. The outer converts a desired *angle* into a desired
*rate*; the inner is the same rate PID that flies acro.

```
want_angle  = stick * MAX_ANGLE_DEG           (30 deg at full stick)
rate_demand = (want_angle - actual_angle) * ANGLE_P      (P = 6 /s)
rate_demand = clamp(rate_demand, +-MAX_RATE)
              |
              v
           rate PID, exactly as in acro
```

Centre the sticks and the demanded angle is zero, so the aircraft levels
itself. The clamp matters: the outer loop must never ask for a rate the
airframe cannot deliver.

Yaw stays rate-controlled in both modes.

## PID

`PID_Update(setpoint, measurement, dt)`, output in mix-authority units where
1.0 is the full throttle range.

### Derivative acts on the measurement, not the error

```c
dRaw = -(measurement - prevMeasurement) / dt;
```

With a constant setpoint these are equivalent - the derivative of the error
is just the negated derivative of the measurement, hence the minus sign.
They diverge the instant the pilot moves a stick: differentiating the error
turns that step change into an enormous spike straight onto the motors
("derivative kick") that has nothing to do with how the aircraft is moving.
Differentiating the measurement responds only to real motion.

### The derivative is low-pass filtered

Differentiation amplifies high-frequency noise, and gyro data is noisy -
badly so once props are spinning. Unfiltered D produces hot motors, audible
buzzing and a twitchy aircraft regardless of gains. The 60 Hz PT1 is what
makes D usable at all.

### The integral is clamped

If the aircraft cannot reach the setpoint - motors saturated, or someone
holding it - the error keeps accumulating and the term grows without bound,
then takes just as long to unwind once control returns. The clamp (0.10, or
10% of throttle range) bounds that.

The controller is also held in `PID_Reset` whenever disarmed or below idle
throttle, so it cannot integrate against the ground and then dump the
accumulated correction into the motors at lift-off.

## The mixer

```
motor_i = throttle + roll*R_i + pitch*P_i + yaw*Y_i
```

| Motor | Position | Spin | Roll | Pitch | Yaw |
|---|---|---|:--:|:--:|:--:|
| M1 | rear-right | CW | -1 | -1 | -1 |
| M2 | front-right | CCW | -1 | +1 | +1 |
| M3 | rear-left | CCW | +1 | -1 | +1 |
| M4 | front-left | CW | +1 | +1 | -1 |

**Roll**: lift the left pair to put the right side down.

**Pitch**: lift the **front** pair to raise the nose. Thrust at the front
rotates the front upward and the nose follows it. This column was inverted
in an early version, which made the controller reinforce pitch disturbances
rather than oppose them.

**Yaw**: a motor spinning clockwise drags the airframe counter-clockwise by
reaction, turning the nose left. So yawing right means speeding up the CCW
pair, M2 and M3.

### Saturation

Four demands built from one throttle plus three corrections will not always
fit in 0..1. The mixer resolves that in a fixed order:

1. **Raise the throttle, but by at most `MIXER_MAX_BOOST` (0.10).** An
   earlier version raised it as far as the correction wanted. At 6% throttle
   a modest roll command silently shoved the average to 37% - a small stick
   input became a large unrequested climb, and the throttle stick lost
   authority.
2. **Lower the throttle freely.** Reducing to fit never produces a climb the
   pilot did not ask for.
3. **Scale whatever still does not fit out of the correction**, all four
   motors together.

Step 3 scales rather than clipping each motor individually, because clipping
distorts the *ratios* between motors - and those ratios are what determine
attitude. A clipped roll command quietly becomes a roll plus an unwanted
yaw.

`Mixer_Apply` returns the fraction of correction it could not deliver, which
lands in `g_mixClipped`.

### Why authority is weak at low throttle

At 6% throttle the motors can only be reduced by 6% before hitting zero, so
there is almost no room for a correction that needs to slow one side. This
is physics, not a bug - and it is why `IDLE_MOTOR` keeps the motors turning
at 4% whenever armed. Stationary motors give the controller no authority at
all, and a stopped ESC takes time to restart.

## Attitude estimation

Angle mode needs absolute roll and pitch. Two imperfect sources:

- **Integrating the gyro** gives a fast, clean angle that **drifts** without
  bound, because tiny bias errors accumulate forever.
- **The accelerometer** senses gravity, so it never drifts - but it is noisy,
  and it cannot distinguish gravity from acceleration.

A complementary filter takes the short-term answer from the first and the
long-term answer from the second:

```
angle = alpha * (angle + rate*dt) + (1-alpha) * accel_angle
alpha = tau / (tau + dt)              tau = 1.0 s
```

Anything faster than tau comes from the gyro; anything slower is pulled back
into line by gravity.

Angles from the accelerometer, in the body frame, where level at rest reads
(0, 0, -1 g):

```
roll  = atan2(-ay, -az)
pitch = atan2(ax, sqrt(ay^2 + az^2))
```

### The accelerometer must be filtered before it is trusted

During a hard climb or sharp turn the accelerometer measures thrust as well
as gravity, so the implied angle is genuinely wrong. The estimator skips the
correction when the magnitude leaves 0.80-1.20 g and coasts on the gyro.

**That test must run on filtered data.** An earlier version tested the raw
magnitude, and prop vibration pushed it outside the band on most samples -
so the correction was rejected almost always, the estimate became a pure
integrating gyro, and it drifted without bound. Angle mode then flew towards
an increasingly wrong idea of level, which presented as an uncontrollable
roll that worsened over a flight.

A 10 Hz low-pass fixes it. The direction of gravity changes at a few Hz even
in aggressive flight; prop vibration lives above 100 Hz. `g_trustRatio`
reports the fraction of samples actually believed - it should be well above
0.8 in flight, and is ~0.999 on the bench.

## Gyro bias

A gyro reads slightly non-zero when perfectly still, and that offset moves
as the part warms.

**The integral term cannot correct this.** If the bias is -0.5 deg/s and the
aircraft rotates at exactly 0.5 deg/s, the gyro reads **zero**. The
controller sees no error, so there is nothing for the integral to accumulate
against. It is not that the correction is too weak - the controller is blind
to it.

Two mechanisms handle it:

1. **Startup calibration** - 1000 samples averaged, with a per-axis min/max
   spread check that fails if anything moved more than ~12 deg/s. An offset
   measured while moving is worse than no offset.
2. **Background tracking** - whenever disarmed and all three axes read under
   2 deg/s, the stored bias walks slowly towards whatever is left after
   correction (about 10 seconds to absorb 1 deg/s). This follows thermal
   drift between flights. It is disabled the moment you arm, because in
   flight it would teach the controller that real rotation is its own error.

### Yaw has no absolute reference

Roll and pitch have gravity, so residual error gets pulled back. Yaw has
nothing - no magnetometer, no GPS. Slow yaw drift in acro is therefore a
property of the setup, not a bug, and Betaflight behaves the same way. Bias
tracking reduces it; only a magnetometer would remove it.

## Arming

```c
if (failsafe || !arm_switch_high)      armed = false;
else if (!armed && throttle < 0.05)    armed = true;
```

A **latch**, not a live condition. Entering the armed state requires the
throttle already down, so flipping the arm switch with the stick up does
nothing until you pull throttle down first. Losing the link or dropping the
switch exits immediately.

Failsafe fires after 250 ms with no RC frame.
