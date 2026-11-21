#pragma once

#include <map>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include "skytrack/tracking/motion.hpp"
#include "skytrack/tracking/sensor_model.hpp"

namespace skytrack {

/// Online sensor registration: estimates how far each sensor's mount is
/// turned from its surveyed heading and pitch, and how far its clock is off.
///
/// Cooperative targets (ADS-B aircraft, Remote ID drones) broadcast their own
/// GNSS position. Whenever a radar, camera or direction finder detects one,
/// the difference between what it reported and what it should have reported
/// for the broadcast position is evidence about its errors. A mount error
/// shifts every angle by the same amount; a clock error shifts angles along
/// the target's direction of motion, in proportion to its speed. Each
/// sensor's (heading, pitch, clock) is a small Kalman filter whose prior is
/// the survey and timing accuracy.
///
/// The tracker uses the estimates twice: to correct each measurement, and to
/// widen its noise by what is still unknown, so an uncalibrated sensor is
/// trusted less than a calibrated one.
///
/// A reference's broadcast position is itself off by its GNSS error, which
/// drifts only slowly. Samples of one reference share it, so the filter
/// carries, per reference, the covariance between its own estimate's error
/// and that GNSS error (a Schmidt or "consider" filter) instead of treating
/// each sample as new. The tracker uses that covariance too: a sensor
/// calibrated against an aircraft's ADS-B inherits part of that ADS-B's
/// error, so it cannot confirm it.
struct RegistrationParams {
  double mount_std_rad = 0.0;               // prior: survey accuracy of heading and pitch
  double clock_std_s = 0.0;                 // prior: timestamp accuracy
  double mount_drift_rad_per_sqrt_s = 0.0;  // random walk, so the estimate can follow slow changes
  double clock_drift_s_per_sqrt_s = 0.0;
  /// Time constant of a reference's GNSS error (first-order Gauss-Markov).
  double reference_correlation_s = 30.0;
  bool learn = true;
};

class Registration {
 public:
  struct Estimate {
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();  // heading (rad), pitch (rad), clock (s)
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    int samples = 0;
    int rejected = 0;  // too far off to be the same target
    std::map<std::string, double> last_sample_s;  // per reference target
    std::optional<double> t_s;                    // time of the last update
    /// Per reference target: Cov(estimate error, the reference's GNSS error
    /// in east, north, up), as of last_sample_s.
    std::map<std::string, Eigen::Matrix3d> cross;
  };

  explicit Registration(RegistrationParams params);

  /// Current estimate for a sensor (the prior if it has seen no reference yet).
  Estimate estimate(const std::string& sensor_id) const;
  /// Corrects a measurement's time and mount, and attaches what is still
  /// unknown. Applying it again replaces the earlier correction.
  void apply(Measurement& m) const;
  /// Learns from measurement m (already corrected by apply) of the target
  /// `reference_id`, whose broadcast state at m.t_s is `reference`; `gnss`
  /// is the part of its position covariance that the target's reports share.
  /// Returns how the step changed the error eps of the sensor's estimate:
  /// eps' = keep eps + terms that only the reference shares (its GNSS error,
  /// the sensor's noise). Nothing if the sample was not used.
  std::optional<Eigen::Matrix3d> observe(const Measurement& m, const Gaussian& reference,
                                         const Eigen::Matrix3d& gnss, const std::string& reference_id);

  const std::map<std::string, Estimate>& estimates() const { return estimates_; }

 private:
  Estimate prior() const;
  /// Cov(error of the sensor's estimate, GNSS error of the reference) at time t;
  /// zero if the sensor never used that reference.
  Eigen::Matrix3d cross(const std::string& sensor_id, const std::string& reference_id, double t) const;

  RegistrationParams params_;
  std::map<std::string, Estimate> estimates_;
};

}  // namespace skytrack
