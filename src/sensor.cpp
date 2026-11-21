#include "skytrack/sensor.hpp"

#include <stdexcept>

namespace skytrack {

Sensor::Sensor(SensorCommon common) : common_(std::move(common)) {
  if (!(common_.rate_hz > 0.0)) {
    throw std::invalid_argument("sensor '" + common_.id + "': rate_hz must be positive");
  }
  const Mount m = true_mount();
  true_position_ = m.position_enu;
  R_true_ = m.R_world_from_body();
}

Mount Sensor::true_mount() const {
  Mount m = common_.mount;
  const SensorErrors& e = common_.errors;
  m.position_enu += e.position_offset_enu;
  m.heading_rad += e.heading_bias_rad;
  m.pitch_rad += e.pitch_bias_rad;
  m.roll_rad += e.roll_bias_rad;
  return m;
}

double Sensor::sensor_clock(double t_true) const {
  return t_true + common_.errors.clock_offset_s + common_.errors.clock_drift_ppm * 1e-6 * t_true;
}

}  // namespace skytrack
