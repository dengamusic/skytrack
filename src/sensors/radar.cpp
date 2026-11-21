#include "skytrack/sensors/radar.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace skytrack {

Radar::Radar(SensorCommon common, RadarParams params) : Sensor(std::move(common)), params_(params) {
  const auto fail = [this](const std::string& what) {
    throw std::invalid_argument("sensor '" + id() + "': " + what);
  };
  if (!(params_.min_range_m >= 0.0 && params_.max_range_m > params_.min_range_m)) fail("need 0 <= min_range_m < max_range_m");
  if (!(params_.fov_az_rad > 0.0)) fail("fov_az_deg must be positive");
  if (!(params_.max_el_rad > params_.min_el_rad)) fail("need min_el_deg < max_el_deg");
  if (!(params_.pfa > 0.0 && params_.pfa < 1.0)) fail("pfa must be in (0, 1)");
  if (!(params_.ref_range_m > 0.0)) fail("ref_range_m must be positive");
  if (params_.doppler_notch_mps >= params_.max_range_rate_mps) fail("doppler_notch_mps must be below max_range_rate_mps");
}

double Radar::snr_db(double range_m, double rcs_dbsm) const {
  return params_.snr_ref_db + rcs_dbsm - 40.0 * std::log10(range_m / params_.ref_range_m);
}

double Radar::detection_probability(double snr) const {
  const double snr_linear = std::pow(10.0, snr / 10.0);
  return std::pow(params_.pfa, 1.0 / (1.0 + snr_linear));
}

bool Radar::in_coverage(const geo::Spherical& body) const {
  if (body.range < params_.min_range_m || body.range > params_.max_range_m) return false;
  if (body.el < params_.min_el_rad || body.el > params_.max_el_rad) return false;
  if (params_.fov_az_rad < 2.0 * geo::kPi && std::abs(body.az) > 0.5 * params_.fov_az_rad) return false;
  return true;
}

double Radar::beam_time(double t_scan, double az_rad) const {
  if (params_.mode == RadarParams::Mode::Staring) return t_scan;
  // Clockwise sweep: the beam reaches body azimuth -x after x / (2 pi) of a period.
  return t_scan + geo::wrap_2pi(-az_rad) / (2.0 * geo::kPi) * period_s();
}

Scan Radar::observe(double t, const World& world, Rng& rng) const {
  const bool rotating = params_.mode == RadarParams::Mode::Rotating;
  Scan scan;
  scan.t_true_s = t;
  scan.t_ready_s = rotating ? t + period_s() : t;

  const double scale = common_.errors.noise_scale;
  const double threshold = -std::log(params_.pfa);  // in units of noise power

  for (const Target& target : world.targets) {
    double t_meas = t;
    if (rotating) {
      // The target moves while the beam comes round; two passes settle the time.
      for (int pass = 0; pass < 2; ++pass) {
        const Vec3 rel = relative_body(target.trajectory.state(t_meas).position);
        t_meas = beam_time(t, std::atan2(rel.y(), rel.x()));
      }
    }
    if (!target.trajectory.active(t_meas)) continue;

    const KinematicState state = target.trajectory.state(t_meas);
    const Vec3 rel = relative_body(state.position);
    const geo::Spherical body = geo::cartesian_to_spherical(rel);
    if (!in_coverage(body)) continue;

    const double range_rate = rel.dot(R_true().transpose() * state.velocity) / body.range;
    if (std::abs(range_rate) < params_.doppler_notch_mps) continue;

    const double snr = snr_db(body.range, target.rcs_dbsm);
    if (!rng.bernoulli(detection_probability(snr))) continue;

    // Received power of a Swerling I target is exponential with mean 1 + SNR.
    // Given that it crossed the threshold, the excess is again exponential.
    const double snr_linear = std::pow(10.0, snr / 10.0);
    const double power = threshold + (1.0 + snr_linear) * rng.exponential();

    RadarDetection d;
    d.range_m = body.range + scale * params_.sigma_range_m * rng.normal();
    d.az_rad = geo::wrap_pi(body.az + scale * params_.sigma_az_rad * rng.normal());
    d.el_rad = body.el + scale * params_.sigma_el_rad * rng.normal();
    d.range_rate_mps = range_rate + scale * params_.sigma_range_rate_mps * rng.normal();
    d.snr_db = 10.0 * std::log10(power);
    scan.detections.push_back(make_detection(t_meas, target.id, d));
  }

  // False alarms: uniform over the coverage, outside the Doppler notch.
  const bool full_circle = params_.fov_az_rad >= 2.0 * geo::kPi;
  const int false_alarms = rng.poisson(params_.false_alarms_per_scan);
  for (int i = 0; i < false_alarms; ++i) {
    RadarDetection d;
    d.range_m = rng.uniform(params_.min_range_m, params_.max_range_m);
    d.az_rad = full_circle ? rng.uniform(-geo::kPi, geo::kPi)
                           : rng.uniform(-0.5 * params_.fov_az_rad, 0.5 * params_.fov_az_rad);
    d.el_rad = rng.uniform(params_.min_el_rad, params_.max_el_rad);
    const double speed = rng.uniform(params_.doppler_notch_mps, params_.max_range_rate_mps);
    d.range_rate_mps = rng.bernoulli(0.5) ? speed : -speed;
    d.snr_db = 10.0 * std::log10(threshold + rng.exponential());  // noise that just crossed the threshold
    scan.detections.push_back(make_detection(beam_time(t, d.az_rad), -1, d));
  }

  if (rotating) {
    std::stable_sort(scan.detections.begin(), scan.detections.end(),
                     [](const Detection& a, const Detection& b) { return a.t_true_s < b.t_true_s; });
  }
  return scan;
}

}  // namespace skytrack
