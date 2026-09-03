#include "helpers/filter.h"

#define PT1_PI 3.14159265358979f

void PT1_Init(PT1_t *f, float cutoffHz, float dt)
{
    /* RC time constant of the equivalent analogue filter, then the discrete
       coefficient k = dt / (RC + dt). */
    const float rc = 1.0f / (2.0f * PT1_PI * cutoffHz);

    f->k     = dt / (rc + dt);
    f->state = 0.0f;
}

float PT1_Apply(PT1_t *f, float input)
{
    f->state += (input - f->state) * f->k;

    return f->state;
}

void PT1_Reset(PT1_t *f, float value)
{
    f->state = value;
}
