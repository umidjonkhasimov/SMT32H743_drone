#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

/* First-order (single-pole) low-pass, the same "PT1" filter Betaflight uses
   for its basic gyro filtering. One multiply and one add per sample.

   Each call moves the stored state a fraction k of the way towards the new
   input. Small k = heavy smoothing but more delay; large k = responsive but
   passes more noise. Phase delay is the real cost - a filter that lags too
   much makes the D term fight the aircraft instead of damping it. */
typedef struct
{
    float state;
    float k;
} PT1_t;

void  PT1_Init(PT1_t *f, float cutoffHz, float dt);
float PT1_Apply(PT1_t *f, float input);
void  PT1_Reset(PT1_t *f, float value);

#endif /* FILTER_H */
