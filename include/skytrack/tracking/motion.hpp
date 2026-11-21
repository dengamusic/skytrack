#pragma once

#include <Eigen/Dense>

/// Nearly-constant-velocity motion model for the tracker.
///
/// State: [east, north, up, v_east, v_north, v_up] in the ENU world frame.
/// Unknown accelerations are modelled as continuous white noise with power
/// spectral density q (m^2/s^3), separately for the horizontal and vertical
/// axes: drones and birds manoeuvre far more sideways than up and down.
namespace skytrack {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

struct ProcessNoise {
  double horizontal_m2_s3 = 4.0;
  double vertical_m2_s3 = 0.5;
};

/// State transition over dt seconds (dt may be negative: that predicts backwards).
inline Mat6 ncv_transition(double dt) {
  Mat6 F = Mat6::Identity();
  F.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
  return F;
}

/// Process noise accumulated over dt seconds, forwards or backwards.
///
/// Forwards, the noise w added to F(dt) x has covariance
///   q [t^3/3, t^2/2; t^2/2, t]  per axis (position, velocity), t = dt.
/// Backwards (dt < 0), x(t - tau) = F(-tau) x(t) - F(-tau) w, whose noise
/// covariance F(-tau) Q(tau) F(-tau)' keeps the diagonal but flips the sign
/// of the position-velocity term: an unknown acceleration that moved the
/// target forward also means its earlier position was further back.
inline Mat6 ncv_process_noise(double dt, const ProcessNoise& q) {
  const double t = dt < 0.0 ? -dt : dt;
  const double t3 = t * t * t;
  const double cross = dt * t / 2.0;  // signed: dt |dt| / 2
  Mat6 Q = Mat6::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const double psd = axis < 2 ? q.horizontal_m2_s3 : q.vertical_m2_s3;
    Q(axis, axis) = psd * t3 / 3.0;
    Q(axis, axis + 3) = psd * cross;
    Q(axis + 3, axis) = psd * cross;
    Q(axis + 3, axis + 3) = psd * t;
  }
  return Q;
}

/// A state estimate and its covariance at a time.
struct Gaussian {
  Vec6 x = Vec6::Zero();
  Mat6 P = Mat6::Identity();
};

/// Predicts a state estimate forward by dt >= 0.
inline Gaussian ncv_predict(const Gaussian& g, double dt, const ProcessNoise& q) {
  const Mat6 F = ncv_transition(dt);
  return {F * g.x, F * g.P * F.transpose() + ncv_process_noise(dt, q)};
}

}  // namespace skytrack
