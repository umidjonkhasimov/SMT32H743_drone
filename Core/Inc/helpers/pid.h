#ifndef PID_H
#define PID_H

#include "helpers/filter.h"
#include <stdint.h>

/* A single-axis rate controller. Setpoint and measurement are both angular
   rates in deg/s; the output is a mix authority in the range roughly -1..+1,
   where 1.0 means "the entire throttle range". */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float iTerm;
    float iLimit;          /* absolute clamp, stops the integral winding up */
    float prevMeasurement;

    PT1_t dLpf;            /* the D term is far too noisy to use unfiltered */
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd,
               float iLimit, float dCutoffHz, float dt);
float PID_Update(PID_t *p, float setpoint, float measurement, float dt);
void  PID_Reset(PID_t *p);

#endif /* PID_H */
