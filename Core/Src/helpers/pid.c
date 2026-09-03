#include "helpers/pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd,
              float iLimit, float dCutoffHz, float dt)
{
    p->kp     = kp;
    p->ki     = ki;
    p->kd     = kd;
    p->iLimit = iLimit;

    PT1_Init(&p->dLpf, dCutoffHz, dt);
    PID_Reset(p);
}

void PID_Reset(PID_t *p)
{
    p->iTerm           = 0.0f;
    p->prevMeasurement = 0.0f;

    PT1_Reset(&p->dLpf, 0.0f);
}

float PID_Update(PID_t *p, float setpoint, float measurement, float dt)
{
    const float error = setpoint - measurement;

    const float pTerm = p->kp * error;

    /* Integral. The clamp is what stops "windup": if the aircraft cannot
       reach the setpoint - motors saturated, or someone holding it - the
       error keeps accumulating and the term grows without bound, then takes
       just as long to unwind once control is restored. */
    p->iTerm += p->ki * error * dt;

    if (p->iTerm >  p->iLimit) { p->iTerm =  p->iLimit; }
    if (p->iTerm < -p->iLimit) { p->iTerm = -p->iLimit; }

    /* Derivative on the *measurement*, not the error.

       With a constant setpoint the two are equivalent (d/dt of the error is
       just the negated d/dt of the measurement, hence the minus sign). They
       differ the instant the pilot moves a stick: differentiating the error
       turns that step change into an enormous spike straight onto the
       motors - "derivative kick" - which has nothing to do with how the
       aircraft is actually moving. Differentiating the measurement responds
       only to real motion. */
    const float dRaw = -(measurement - p->prevMeasurement) / dt;

    p->prevMeasurement = measurement;

    const float dTerm = p->kd * PT1_Apply(&p->dLpf, dRaw);

    return pTerm + p->iTerm + dTerm;
}
