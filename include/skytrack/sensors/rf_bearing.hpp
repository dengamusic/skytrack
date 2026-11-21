#pragma once

#include "skytrack/sensor.hpp"

namespace skytrack {

struct RfBearingParams {
  double max_range_m = 3000.0;
  double fov_az_rad = 2.0 * geo::kPi;
  double sigma_az_rad = geo::deg2rad(3.0);  // stated 1-sigma
  double pd = 0.9;                          // per report, for an emitter in range
  double snr_ref_db = 25.0;                 // SNR at ref_range_m, falling with range^2
  double ref_range_m = 1000.0;
  double false_alarms_per_report = 0.05;
  double signature_id_prob = 0.8;           // chance the emitter type is recognised
};

/// Passive RF direction finder. It hears the control link of drones that
/// emit one and reports only a bearing: no range, no elevation.
class RfBearingSensor final : public Sensor {
 public:
  RfBearingSensor(SensorCommon common, RfBearingParams params);

  SensorType type() const override { return SensorType::RfBearing; }
  Scan observe(double t, const World& world, Rng& rng) const override;

  const RfBearingParams& params() const { return params_; }

 private:
  RfBearingParams params_;
};

}  // namespace skytrack
