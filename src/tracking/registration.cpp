#include "skytrack/tracking/registration.hpp"

#include <algorithm>

namespace skytrack {

Registration::Registration(RegistrationParams params) : params_(params) {}

Registration::Estimate Registration::prior() const {
  Estimate e;
  const double m2 = params_.mount_std_rad * params_.mount_std_rad;
  e.P.diagonal() << m2, m2, params_.clock_std_s * params_.clock_std_s;
  return e;
}

Registration::Estimate Registration::estimate(const std::string& sensor_id) const {
  const auto it = estimates_.find(sensor_id);
  return it == estimates_.end() ? prior() : it->second;
}

void Registration::apply(Measurement& m) const {
  if (!m.sensor->measures_angles()) return;
  const Estimate e = estimate(m.sensor->id());
  if (!m.t_stamp_s) m.t_stamp_s = m.t_s;
  m.mount_correction = e.bias.head<2>();
  m.t_s = *m.t_stamp_s - e.bias[2];  // a clock running ahead stamps measurements too late
  m.registration_covariance = e.P;
}

Eigen::Matrix3d Registration::cross(const std::string& sensor_id, const std::string& reference_id, double t) const {
  const auto e = estimates_.find(sensor_id);
  if (e == estimates_.end()) return Eigen::Matrix3d::Zero();
  const auto x = e->second.cross.find(reference_id);
  if (x == e->second.cross.end()) return Eigen::Matrix3d::Zero();
  const double age = std::abs(t - e->second.last_sample_s.at(reference_id));
  return x->second * std::exp(-age / params_.reference_correlation_s);
}

std::optional<Eigen::Matrix3d> Registration::observe(const Measurement& m, const Gaussian& reference,
                                                        const Eigen::Matrix3d& gnss, const std::string& reference_id) {
  if (!params_.learn || !m.sensor->measures_angles()) return std::nullopt;
  const SensorModel& model = *m.sensor;
  auto [it, inserted] = estimates_.try_emplace(model.id(), prior());
  Estimate& e = it->second;

  // The errors may drift: what was learned grows less certain with time.
  if (e.t_s && m.t_s > *e.t_s) {
    const double dt = m.t_s - *e.t_s;
    const double md = params_.mount_drift_rad_per_sqrt_s;
    const double cd = params_.clock_drift_s_per_sqrt_s;
    e.P.diagonal() += Eigen::Vector3d(md * md, md * md, cd * cd) * dt;
  }
  if (!e.t_s || m.t_s > *e.t_s) e.t_s = m.t_s;

  // m's time was already corrected with the clock estimate; its mount is
  // evaluated with the current estimate.
  Measurement probe = m;
  probe.mount_correction = e.bias.head<2>();
  const auto zhat = model.predict(reference.x, probe);
  const Eigen::MatrixXd J = registration_jacobian(model, reference.x, probe);
  const Eigen::MatrixXd H = measurement_jacobian(model, reference.x, probe);
  if (!zhat || J.size() == 0 || H.size() == 0) return std::nullopt;

  // Residual r = -J eps - H b - H w + v, with eps the estimate's error, b
  // the reference's GNSS error (shared with its other reports), w the rest
  // of the reference's error and v the sensor's noise. X = Cov(eps, b).
  Eigen::Matrix3d X = cross(model.id(), reference_id, m.t_s);
  Eigen::MatrixXd reference_rest = reference.P;
  reference_rest.topLeftCorner<3, 3>() -= gnss;
  const Eigen::MatrixXd Hp = H.leftCols<3>();  // the GNSS error moves the position only
  const Eigen::MatrixXd R = model.noise(m) + H * reference_rest * H.transpose();
  const Eigen::MatrixXd JXH = J * X * Hp.transpose();
  const Eigen::VectorXd r = model.residual(m.z, *zhat);
  const Eigen::MatrixXd S = J * e.P * J.transpose() + Hp * gnss * Hp.transpose() + R + JXH + JXH.transpose();
  const auto S_ldlt = S.ldlt();
  if (r.dot(S_ldlt.solve(r)) > gate_threshold(r.size())) {
    ++e.rejected;
    return std::nullopt;
  }
  // K = -Cov(eps, r) S^-1; the error after the update is
  // (I - K J) eps - K H b - K H w + K v.
  const Eigen::MatrixXd K = S_ldlt.solve((J * e.P + Hp * X.transpose())).transpose();
  const Eigen::Matrix3d IKJ = Eigen::Matrix3d::Identity() - K * J;
  const Eigen::Matrix3d KHp = K * Hp;
  const Eigen::Matrix3d shared = IKJ * X * KHp.transpose();
  e.bias += K * r;
  e.P = IKJ * e.P * IKJ.transpose() + K * R * K.transpose() + KHp * gnss * KHp.transpose() - shared -
        shared.transpose();
  e.P = 0.5 * (e.P + e.P.transpose()).eval();
  // Every reference's GNSS error is independent of this sample's other terms.
  for (auto& [id, other] : e.cross) {
    if (id != reference_id) other = IKJ * cross(model.id(), id, m.t_s);
  }
  for (auto& [id, when] : e.last_sample_s) {
    if (id != reference_id) when = m.t_s;
  }
  e.cross[reference_id] = IKJ * X - KHp * gnss;
  e.last_sample_s[reference_id] = m.t_s;
  ++e.samples;
  return IKJ;
}

}  // namespace skytrack
