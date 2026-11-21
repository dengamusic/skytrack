#pragma once

#include "skytrack/sensor.hpp"

namespace skytrack {

struct RadarParams {
  enum class Mode { Staring, Rotating };
  Mode mode = Mode::Staring;

  double min_range_m = 50.0;
  double max_range_m = 5000.0;
  double fov_az_rad = 2.0 * geo::kPi;  // full width, centred on the boresight
  double min_el_rad = geo::deg2rad(-2.0);
  double max_el_rad = geo::deg2rad(30.0);

  // Stated 1-sigma accuracy, as on a data sheet. The real noise is this
  // times SensorErrors::noise_scale.
  double sigma_range_m = 5.0;
  double sigma_az_rad = geo::deg2rad(0.5);
  double sigma_el_rad = geo::deg2rad(1.0);
  double sigma_range_rate_mps = 0.3;

  // Detection. SNR of a 0 dBsm target at ref_range_m; it falls with range^4
  // and rises with the target's radar cross section.
  double snr_ref_db = 50.0;
  double ref_range_m = 1000.0;
  double pfa = 1e-6;  // false-alarm probability per cell; sets the threshold

  double doppler_notch_mps = 0.0;  // |range rate| below this is filtered out with the ground clutter
  double false_alarms_per_scan = 1.0;
  double max_range_rate_mps = 50.0;  // spread of false-alarm range rates
};

/// Radar reporting range, azimuth, elevation and range rate.
///
/// A staring radar sees its whole field of view at once. A rotating radar
/// sweeps clockwise once per period, starting at its boresight, so each
/// target is measured at the moment the beam passes it and the scan is only
/// sent once the sweep finishes.
class Radar final : public Sensor {
 public:
  Radar(SensorCommon common, RadarParams params);

  SensorType type() const override { return SensorType::Radar; }
  Scan observe(double t, const World& world, Rng& rng) const override;

  const RadarParams& params() const { return params_; }
  double snr_db(double range_m, double rcs_dbsm) const;
  /// Swerling I target, square-law detector: Pd = Pfa^(1 / (1 + SNR)).
  double detection_probability(double snr_db) const;
  bool in_coverage(const geo::Spherical& body) const;
  /// When the beam points at body azimuth az during the scan starting at t_scan.
  double beam_time(double t_scan, double az_rad) const;

 private:
  RadarParams params_;
};

}  // namespace skytrack
