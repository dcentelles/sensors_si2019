#ifndef INCLUDE_SENSORS2019_UTILS_HPP
#define INCLUDE_SENSORS2019_UTILS_HPP
#include <math.h>

namespace sensors_si2019 {
namespace utils {

static constexpr double range_yaw = M_PI * 2;

int GetDiscreteRot(double rad) {
  int degrees = std::round(rad * 180.0 / M_PI); // conversion to degrees
  if (degrees < 0)
    degrees += 360; // convert negative to positive angles

  return degrees;
}
double GetContinuousRot(int deg) { return deg / 180. * M_PI; }
} // namespace utils
} // namespace sensors_si2019
#endif // UTILS_HPP
