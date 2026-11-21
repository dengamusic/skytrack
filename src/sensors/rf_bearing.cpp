#include "skytrack/sensors/rf_bearing.hpp"

#include <cmath>
#include <stdexcept>

namespace skytrack {

RfBearingSensor::RfBearingSensor(SensorCommon common, RfBearingParams params)
    : Sensor(std::move(common)), params_(params) {
  if (!(params_.max_range_m > 0.0 && params_.ref_range_m > 0.0)) {
    throw std::invalid_argument("sensor '" + id() + "': ranges must be positive");
  }
}

Scan RfBearingSensor::observe(double t, const World& world, Rng& rng) const {
  Scan scan;
  scan.t_true_s = t;
  scan.t_ready_s = t;
  const double scale = common_.errors.noise_scale;
  const bool full_circle = params_.fov_az_rad >= 2.0 * geo::kPi;

  for (const Target& target : world.targets) {
    if (target.rf_signature.empty() || !target.trajectory.active(t)) continue;
    const Vec3 rel = relative_body(target.trajectory.state(t).position);
    const double range = rel.norm();
    if (range > params_.max_range_m) continue;
    const double az = std::atan2(rel.y(), rel.x());
    if (!full_circle && std::abs(az) > 0.5 * params_.fov_az_rad) continue;
    if (!rng.bernoulli(params_.pd)) continue;

    BearingDetection d;
    d.az_rad = geo::wrap_pi(az + scale * params_.sigma_az_rad * rng.normal());
    d.snr_db = params_.snr_ref_db - 20.0 * std::log10(range / params_.ref_range_m) + rng.normal(0.0, 2.0);
    if (rng.bernoulli(params_.signature_id_prob)) d.signature = target.rf_signature;
    scan.detections.push_back(make_detection(t, target.id, d));
  }

  const double weakest_snr = params_.snr_ref_db - 20.0 * std::log10(params_.max_range_m / params_.ref_range_m);
  const int false_alarms = rng.poisson(params_.false_alarms_per_report);
  for (int i = 0; i < false_alarms; ++i) {
    BearingDetection d;
    d.az_rad = full_circle ? rng.uniform(-geo::kPi, geo::kPi)
                           : rng.uniform(-0.5 * params_.fov_az_rad, 0.5 * params_.fov_az_rad);
    d.snr_db = weakest_snr + std::abs(rng.normal(0.0, 2.0));
    scan.detections.push_back(make_detection(t, -1, d));
  }
  return scan;
}

}  // namespace skytrack
