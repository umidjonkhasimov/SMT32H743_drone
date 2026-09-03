#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

#define MIXER_MOTORS 4

/* Combines a throttle level with roll/pitch/yaw authority into four motor
   demands, each 0.0..1.0.

   throttle  0.0 .. 1.0
   roll      positive rolls right side down
   pitch     positive pitches nose up
   yaw       positive yaws nose right

   Returns the amount of authority that had to be given up to keep every
   motor inside its range - 0.0 when nothing was clipped. Useful to watch:
   sustained non-zero means the aircraft is asking for more than it has. */
float Mixer_Apply(float throttle, float roll, float pitch, float yaw,
                  float out[MIXER_MOTORS]);

#endif /* MIXER_H */
