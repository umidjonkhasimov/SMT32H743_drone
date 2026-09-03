#include "helpers/mixer.h"

/* Quad-X, motors ordered M1..M4 to match motors[0..3] in main.

       M4   M2        nose ^
         \ /          M1 = rear-right  (CW)   M2 = front-right (CCW)
         / \          M3 = rear-left   (CCW)  M4 = front-left  (CW)
       M3   M1

   Roll:  lift the left pair to put the right side down.
   Pitch: lift the FRONT pair to raise the nose. Thrust at the front rotates
          the front upwards - the nose follows it. Getting this backwards
          makes the controller reinforce a pitch disturbance instead of
          opposing it, which diverges into a flip.
   Yaw:   a CW motor drags the airframe CCW, turning the nose left, so
          yawing right means speeding up the CCW pair (M2, M3). */
/* Most the mixer may raise the throttle above what the pilot asked for,
   in order to keep roll/pitch/yaw authority at low throttle. */
#define MIXER_MAX_BOOST 0.10f

static const float MIX[MIXER_MOTORS][3] =
{
    /*  roll   pitch   yaw  */
    { -1.0f, -1.0f, -1.0f },   /* M1 rear-right,  CW  */
    { -1.0f,  1.0f,  1.0f },   /* M2 front-right, CCW */
    {  1.0f, -1.0f,  1.0f },   /* M3 rear-left,   CCW */
    {  1.0f,  1.0f, -1.0f },   /* M4 front-left,  CW  */
};

float Mixer_Apply(float throttle, float roll, float pitch, float yaw,
                  float out[MIXER_MOTORS])
{
    float mix[MIXER_MOTORS];
    float lo = 0.0f;
    float hi = 0.0f;

    for (int i = 0; i < MIXER_MOTORS; i++)
    {
        mix[i] = roll  * MIX[i][0]
               + pitch * MIX[i][1]
               + yaw   * MIX[i][2];

        if (i == 0 || mix[i] < lo) { lo = mix[i]; }
        if (i == 0 || mix[i] > hi) { hi = mix[i]; }
    }

    float t = throttle;

    /* The correction needs headroom below the throttle as well as above it.
       When there is not enough below, we may lift the throttle a little to
       make room - but only a little. Lifting it as far as the correction
       wants turns a small stick input into a large unrequested climb, and
       leaves the pilot's throttle stick doing nothing. */
    if (t + lo < 0.0f)
    {
        float boost = -lo - t;

        if (boost > MIXER_MAX_BOOST) { boost = MIXER_MAX_BOOST; }

        t += boost;
    }

    /* Coming down is always allowed: reducing throttle to fit never produces
       a climb the pilot did not ask for. */
    if (t + hi > 1.0f)
    {
        t = 1.0f - hi;

        if (t < 0.0f) { t = 0.0f; }
    }

    /* Whatever still does not fit, take out of the correction - scaling all
       four together so the ratios between them survive. Clipping motors
       individually would distort the correction and turn a roll command
       into a roll plus an unwanted yaw. */
    float need = 0.0f;

    if (t + lo < 0.0f)      { need = -(t + lo); }
    if (t + hi - 1.0f > need) { need = t + hi - 1.0f; }

    const float amp = (-lo > hi) ? -lo : hi;
    float scale = 1.0f;

    if (need > 0.0f && amp > 0.0f)
    {
        scale = (amp - need) / amp;

        if (scale < 0.0f) { scale = 0.0f; }
    }

    for (int i = 0; i < MIXER_MOTORS; i++)
    {
        float v = t + mix[i] * scale;

        if (v < 0.0f) { v = 0.0f; }
        if (v > 1.0f) { v = 1.0f; }

        out[i] = v;
    }

    /* Fraction of the requested correction that could not be delivered. */
    return 1.0f - scale;
}
