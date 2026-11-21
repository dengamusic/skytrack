#pragma once

#include <string>

#include "skytrack/sensor.hpp"

namespace skytrack {

struct CooperativeParams {
  std::string protocol = "adsb";  // which broadcasts it decodes: "adsb" or "remote_id"
  double max_range_m = 250000.0;
  double reception_prob = 0.9;    // per target per report
  double velocity_std_mps = 0.5;
};

/// Receiver for positions that targets broadcast about themselves: an ADS-B
/// receiver, a Remote ID receiver, or an online feed such as OpenSky (same
/// data, much later and less often). Reports are WGS84, not ENU, and each
/// one is stamped with when it was received, not when the batch was sent.
class CooperativeReceiver final : public Sensor {
 public:
  CooperativeReceiver(SensorCommon common, CooperativeParams params);

  SensorType type() const override { return SensorType::Cooperative; }
  Scan observe(double t, const World& world, Rng& rng) const override;

  const CooperativeParams& params() const { return params_; }

 private:
  CooperativeParams params_;
};

}  // namespace skytrack
