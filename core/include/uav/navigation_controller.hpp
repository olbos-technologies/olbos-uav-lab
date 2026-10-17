#pragma once
#include "uav/types.hpp"
namespace uav {
inline Enu step_toward(Enu from, Enu to, double dt, double speed) {
  if (!finite(from) || !finite(to) || !std::isfinite(dt) || !std::isfinite(speed) || dt <= 0 ||
      dt > 0.5 || speed <= 0)
    throw std::invalid_argument("invalid trajectory input");
  double d = distance(from, to);
  double k = d > 0 ? std::min(1.0, speed * dt / d) : 0;
  return {from.east + k * (to.east - from.east), from.north + k * (to.north - from.north),
          from.up + k * (to.up - from.up)};
}
} // namespace uav
