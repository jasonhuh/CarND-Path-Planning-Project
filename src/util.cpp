#include "util.h"
#include "settings.h"
#include <math.h>

double get_d(int lane) {
    if (lane < 0 || lane >= NUM_LANES) {
        return 0.0;
    }
    return LANE_WIDTH * (0.5 + (double)lane);
}

int get_lane(double d) {
  for (int i = 0; i < NUM_LANES; ++i) {
    if ((d >= i * LANE_WIDTH) && (d < (i + 1) * LANE_WIDTH)) {
      return i;
    }
  }
  return -1;
}
