#include "skytrack/tracking/tracker.hpp"

#include "skytrack/tracking/covariance_intersection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "skytrack/tracking/assignment.hpp"

namespace skytrack {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kMinVariance = 1e-4;  // m^2, (m/s)^2: floor when repairing a covariance
constexpr double kRecentSensorsS = 5.0;     // a report lists the sensors that updated the track this recently
constexpr double kSameIdentityCost = 0.01;  // a broadcast carrying the track's identity wins against anonymous ones

bool identities_conflict(const std::string& a, const std::string& b) { return !a.empty() && !b.empty() && a != b; }

Gaussian predicted(const Gaussian& g, double dt, const ProcessNoise& q) { return dt > 0.0 ? ncv_predict(g, dt, q) : g; }

/// Moves an estimate forward or backward in time; either way it gets less certain.
Gaussian shifted(const Gaussian& g, double dt, const ProcessNoise& q) {
  const Mat6 F = ncv_transition(dt);
  return {F * g.x, F * g.P * F.transpose() + ncv_process_noise(dt, q)};
}


RegistrationParams registration_params(const TrackerConfig& cfg) {
  constexpr double kSecondsPerHour = 3600.0;
  RegistrationParams p;
  p.learn = cfg.registration;
  p.mount_std_rad = geo::deg2rad(cfg.mount_accuracy_deg);
  p.clock_std_s = cfg.clock_accuracy_s;
  p.mount_drift_rad_per_sqrt_s = geo::deg2rad(cfg.mount_drift_deg_per_sqrt_h) / std::sqrt(kSecondsPerHour);
  p.clock_drift_s_per_sqrt_s = cfg.clock_drift_s_per_sqrt_h / std::sqrt(kSecondsPerHour);
  p.reference_correlation_s = cfg.broadcast_error_correlation_s;
  return p;
}

using Mat63 = Eigen::Matrix<double, 6, 3>;

Mat63 cross_of(const ConsiderCross& c, std::string_view key) {
  const auto it = c.find(key);
  return it == c.end() ? Mat63::Zero() : it->second;
}

/// Consider covariances carried dt from the track's time. A GNSS error
/// forgets itself with time constant tau; a registration error stays. A
/// measurement older than the track (dt < 0) leaves the state where it is.
ConsiderCross carried(const ConsiderCross& c, double dt, double tau) {
  const Mat6 F = dt > 0.0 ? ncv_transition(dt) : Mat6::Identity();
  const double kept = tau > 0.0 ? std::exp(-std::abs(dt) / tau) : 0.0;
  ConsiderCross out;
  for (const auto& [key, C] : c) out[key] = F * C * (key == kGnssError ? kept : 1.0);
  return out;
}

/// Latest time any sensor other than `sensor_id` updated the track.
double last_update_except(const Track& track, const std::string& sensor_id) {
  double latest = -kInf;
  for (const auto& [id, when] : track.last_update_by_sensor) {
    if (id != sensor_id) latest = std::max(latest, when);
  }
  return latest;
}

/// Extended Kalman update with one scalar pseudo-measurement z = h(x) +- sqrt(var),
/// skipped if its normalised innovation squared exceeds gate_chi2. Returns
/// that statistic.
/// `cross`, the covariance between g's error and something the
/// pseudo-measurement does not involve, is updated along.
template <typename H>
double scalar_update(Gaussian& g, H h, double z, double var, double gate_chi2 = kInf,
                     ConsiderCross* cross = nullptr) {
  constexpr double kStep = 1e-2;
  Eigen::Matrix<double, 1, 6> J;
  for (int k = 0; k < 6; ++k) {
    Vec6 plus = g.x;
    Vec6 minus = g.x;
    plus[k] += kStep;
    minus[k] -= kStep;
    J[k] = (h(plus) - h(minus)) / (2.0 * kStep);
  }
  const double S = (J * g.P * J.transpose())(0, 0) + var;
  const double innovation = z - h(g.x);
  const double nis = innovation * innovation / S;
  if (nis > gate_chi2) return nis;
  const Vec6 K = g.P * J.transpose() / S;
  const Mat6 IKJ = Mat6::Identity() - K * J;
  g.x += K * innovation;
  g.P = IKJ * g.P * IKJ.transpose() + K * var * K.transpose();
  if (cross) {
    for (auto& [key, C] : *cross) C = IKJ * C;
  }
  return nis;
}

}  // namespace

std::string_view to_string(TrackStatus status) {
  return status == TrackStatus::Confirmed ? "confirmed" : "tentative";
}

std::string_view to_string(AssociationOutcome outcome) {
  switch (outcome) {
    case AssociationOutcome::Updated: return "updated";
    case AssociationOutcome::Initiated: return "initiated";
    case AssociationOutcome::Unassigned: return "unassigned";
    case AssociationOutcome::TooOld: return "too_old";
  }
  return "unknown";
}

std::string Track::classification() const {
  std::string best = "unknown";
  double best_votes = 0.0;
  for (const auto& [cls, votes] : class_votes) {
    if (votes > best_votes) {
      best = cls;
      best_votes = votes;
    }
  }
  return best_votes >= 1.0 ? best : "unknown";
}

double Track::rcs_dbsm(double fallback) const {
  constexpr int kMinSamples = 3;
  if (rcs_samples < kMinSamples) return fallback;
  // Floor at 1e-4 m^2 (-40 dBsm): a few weak samples must not make the target invisible.
  return 10.0 * std::log10(std::max(rcs_sum_m2 / rcs_samples, 1e-4));
}

std::string Track::emitter() const {
  std::string best;
  int best_count = 0;
  for (const auto& [name, count] : emitter_counts) {
    if (count > best_count) {
      best = name;
      best_count = count;
    }
  }
  return best;
}

// ---- intake ---------------------------------------------------------------------

Tracker::Tracker(std::shared_ptr<const SensorSetup> setup, TrackerConfig config)
    : setup_(std::move(setup)),
      cfg_(std::move(config)),
      output_period_s_(1.0 / cfg_.output_rate_hz),
      latest_arrival_s_(-kInf),
      processed_until_s_(-kInf),
      registration_(registration_params(cfg_)) {}

void Tracker::ingest(const SensorMessage& msg) {
  const auto started = std::chrono::steady_clock::now();
  const SensorModel* sensor = setup_->find(msg.sensor_id);
  if (sensor == nullptr || !std::isfinite(msg.t_arrival_s)) {
    ++stats_.rejected_messages;  // a field system logs bad input and carries on
    return;
  }
  ++stats_.messages;
  latest_arrival_s_ = std::max(latest_arrival_s_, msg.t_arrival_s);
  const double clock_bias_s = registration_.estimate(sensor->id()).bias[2];
  const double data_end_s = sensor->scan_end_time(msg) - clock_bias_s;
  note_health(*sensor, msg, data_end_s);

  std::vector<Measurement> measurements;
  measurements.reserve(msg.detections.size());
  for (const Detection& d : msg.detections) {
    Measurement m = sensor->convert(d, msg);
    if (!m.z.allFinite() || !std::isfinite(m.t_s)) {
      ++stats_.rejected_measurements;
      ++health_[sensor->id()].rejected;
      continue;
    }
    registration_.apply(m);  // before queueing: the clock correction moves its time
    measurements.push_back(std::move(m));
  }
  stats_.measurements += measurements.size();
  std::stable_sort(measurements.begin(), measurements.end(),
                   [](const Measurement& a, const Measurement& b) { return a.t_s < b.t_s; });

  // A rotating radar's scan is handled in short sectors, each close to the
  // time it was measured. Everything else is one batch per message.
  const auto* radar = dynamic_cast<const RadarModel*>(sensor);
  const double sector_s = radar != nullptr && radar->rotating() ? cfg_.radar_sector_s : kInf;
  const auto new_batch = [&] {
    Event e;
    e.kind = Event::Kind::Batch;
    e.sensor = sensor;
    e.seq = msg.seq;
    return e;
  };
  Event batch = new_batch();
  for (Measurement& m : measurements) {
    if (!batch.measurements.empty() && m.t_s - batch.measurements.front().t_s > sector_s) {
      enqueue(std::move(batch));
      batch = new_batch();
    }
    batch.t_s = m.t_s;  // sorted, so the last one is the latest
    batch.measurements.push_back(std::move(m));
  }
  if (!batch.measurements.empty()) enqueue(std::move(batch));

  // A radar scan that ends also says what it did not see.
  if (radar != nullptr) {
    Event end;
    end.kind = Event::Kind::ScanEnd;
    end.sensor = sensor;
    end.seq = msg.seq;
    // Same clock correction as its detections, so the scan never ends before them.
    end.t_s = data_end_s;
    enqueue(std::move(end));
  }
  release(latest_arrival_s_ - cfg_.reorder_window_s);
  const std::chrono::duration<float, std::micro> took = std::chrono::steady_clock::now() - started;
  ingest_time_us_.push_back(took.count());
}

void Tracker::note_health(const SensorModel& sensor, const SensorMessage& msg, double data_end_s) {
  SensorHealth& h = health_[sensor.id()];
  if (h.messages == 0) h.first_arrival_s = msg.t_arrival_s;
  ++h.messages;
  h.detections += msg.detections.size();
  h.last_arrival_s = std::max(h.last_arrival_s, msg.t_arrival_s);
  if (h.last_seq) {
    if (msg.seq > *h.last_seq + 1) h.lost_messages += msg.seq - *h.last_seq - 1;
    if (msg.seq <= *h.last_seq) ++h.out_of_order;
  }
  if (!h.last_seq || msg.seq > *h.last_seq) h.last_seq = msg.seq;
  const double latency = msg.t_arrival_s - data_end_s;
  h.latency_sum_s += latency;
  h.latency_max_s = std::max(h.latency_max_s, latency);
  if (data_end_s < processed_until_s_) ++h.late_messages;
}

void Tracker::finish() {
  release(kInf);
  if (std::isfinite(processed_until_s_)) publish_before(processed_until_s_, true);
}

void Tracker::enqueue(Event event) {
  const auto key = std::make_pair(event.t_s, next_order_++);
  queue_.emplace(key, std::move(event));
}

void Tracker::release(double up_to) {
  while (!queue_.empty() && queue_.begin()->first.first <= up_to) {
    auto node = queue_.extract(queue_.begin());
    Event& event = node.mapped();
    publish_before(event.t_s, false);
    if (event.kind == Event::Kind::Batch) {
      process_batch(event);
    } else {
      process_scan_end(event);
    }
    processed_until_s_ = std::max(processed_until_s_, event.t_s);
  }
}

// ---- association --------------------------------------------------------------------

std::optional<Tracker::Innovation> Tracker::innovate(const Track& track, const Measurement& m) const {
  const double dt = m.t_s - track.t_s;
  if (dt < -cfg_.max_measurement_age_s) return std::nullopt;

  // A measurement older than the track is compared with the track predicted
  // back to its time; the process noise over that gap is added to the
  // measurement noise. The update then corrects the current state.
  Innovation in;
  in.late = dt < 0.0;
  const double back = in.late ? -dt : 0.0;
  in.prior = predicted(track.state, dt, cfg_.process_noise);
  in.t_ref = in.late ? track.t_s : m.t_s;

  const Mat6 F_back = ncv_transition(-back);
  const Vec6 x_at_measurement = F_back * in.prior.x;
  if (!m.sensor->can_observe(x_at_measurement)) return std::nullopt;
  const auto zhat = m.sensor->predict(x_at_measurement, m);
  if (!zhat) return std::nullopt;
  const Eigen::MatrixXd H_m = measurement_jacobian(*m.sensor, x_at_measurement, m);
  if (H_m.size() == 0) return std::nullopt;

  in.H = H_m * F_back;
  in.R = m.sensor->noise(m);
  in.cross = carried(track.consider, dt, cfg_.broadcast_error_correlation_s);
  in.G = Eigen::MatrixXd::Zero(m.z.size(), 3);
  if (m.sensor->type() == SensorType::Cooperative) {
    // The report's position carries the target's GNSS error b, which its
    // other reports (and so the track) share, plus jitter.
    const auto& coop = static_cast<const CooperativeModel&>(*m.sensor);
    in.consider = kGnssError;
    in.sigma = coop.bias_covariance(m);
    in.R = coop.white_noise(m);
    in.G.topRows<3>() = Eigen::Matrix3d::Identity();
  } else if (!m.registration_covariance.isZero()) {
    // The sensor's mount and clock were corrected with estimates that are
    // off by eps, the same for all its detections: nu gains -J eps.
    const Eigen::MatrixXd J = registration_jacobian(*m.sensor, x_at_measurement, m);
    if (J.size() == 0) return std::nullopt;
    if (cfg_.consider_registration) {
      in.consider = m.sensor->id();
      in.sigma = m.registration_covariance;
      in.G = -J;
    } else {
      in.R += J * m.registration_covariance * J.transpose();
    }
  }
  // Noise of predicting the track back to the measurement: note the minus sign.
  if (in.late) in.R += H_m * ncv_process_noise(-back, cfg_.process_noise) * H_m.transpose();
  // Innovation nu = -H e + G q + w, with e the track's error and Cov(e, q) = Cq.
  const Eigen::MatrixXd H_Cq_G = in.H * cross_of(in.cross, in.consider) * in.G.transpose();
  in.S = in.H * in.prior.P * in.H.transpose() + in.R + in.G * in.sigma * in.G.transpose() - H_Cq_G -
         H_Cq_G.transpose();
  in.nu = m.sensor->residual(m.z, *zhat);
  const auto S_ldlt = in.S.ldlt();
  // The consider covariances are approximate (sensors calibrated against
  // the same aircraft are treated as independent), so S can in rare cases
  // come out indefinite. Such a pairing is not trusted.
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive() || S_ldlt.vectorD().minCoeff() <= 0.0) {
    ++stats_.inconsistent_innovations;
    return std::nullopt;
  }
  in.d2 = in.nu.dot(S_ldlt.solve(in.nu));
  if (!std::isfinite(in.d2)) return std::nullopt;
  return in;
}

void Tracker::process_batch(Event& event) {
  // Registration may have learned more while this batch waited in the
  // reorder buffer. Correct with what it knows now: the tracks' consider
  // covariances describe the error of the current estimate, not of the one
  // the batch was queued with.
  for (Measurement& m : event.measurements) registration_.apply(m);
  // A batch that arrived after newer data was processed may still start a
  // track, as long as it is not much older: with a short reorder window, a
  // rotating radar's scans always arrive "late", and they are what starts
  // tracks. Much older data (an online feed seconds behind) only updates.
  const bool may_initiate = event.t_s >= processed_until_s_ - cfg_.late_initiation_s;
  std::vector<std::size_t> remaining(event.measurements.size());
  for (std::size_t i = 0; i < remaining.size(); ++i) remaining[i] = i;

  // Confirmed tracks choose first, so a fresh tentative track with a wide
  // gate cannot steal a measurement from an established one.
  for (const TrackStatus stage : {TrackStatus::Confirmed, TrackStatus::Tentative}) {
    std::vector<std::size_t> candidates;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (tracks_[i].status == stage) candidates.push_back(i);
    }
    if (candidates.empty() || remaining.empty()) continue;

    const auto rows = static_cast<Eigen::Index>(remaining.size());
    const auto cols = static_cast<Eigen::Index>(candidates.size());
    Eigen::MatrixXd cost = Eigen::MatrixXd::Constant(rows, cols, kForbidden);
    Eigen::VectorXd leave(rows);
    std::vector<std::optional<Innovation>> innovations(remaining.size() * candidates.size());

    for (Eigen::Index r = 0; r < rows; ++r) {
      const Measurement& m = event.measurements[remaining[static_cast<std::size_t>(r)]];
      const double gate = gate_threshold(m.z.size());
      leave[r] = gate;
      for (Eigen::Index c = 0; c < cols; ++c) {
        const Track& track = tracks_[candidates[static_cast<std::size_t>(c)]];
        if (identities_conflict(track.identity, m.identity)) continue;
        auto in = innovate(track, m);
        if (!in) continue;
        const bool same_identity = !m.identity.empty() && m.identity == track.identity;
        if (in->d2 > gate * (same_identity ? cfg_.same_identity_gate_scale : 1.0)) continue;
        cost(r, c) = same_identity ? in->d2 * kSameIdentityCost : in->d2;
        innovations[static_cast<std::size_t>(r * cols + c)] = std::move(in);
      }
    }

    const std::vector<int> assigned = solve_assignment_with_misses(cost, leave);
    std::vector<std::size_t> left_over;
    for (Eigen::Index r = 0; r < rows; ++r) {
      const std::size_t mi = remaining[static_cast<std::size_t>(r)];
      const int c = assigned[static_cast<std::size_t>(r)];
      if (c < 0) {
        left_over.push_back(mi);
        continue;
      }
      update(tracks_[candidates[static_cast<std::size_t>(c)]], event.measurements[mi],
             *innovations[static_cast<std::size_t>(r * cols + c)], event.seq);
    }
    remaining = std::move(left_over);
  }

  for (const std::size_t mi : remaining) {
    const Measurement& m = event.measurements[mi];
    if (may_initiate) {
      if (const auto g = m.sensor->initiate(m, cfg_)) {
        start_track(m, *g, event.seq);
        continue;
      }
    }
    const bool too_old = m.t_s < processed_until_s_ - cfg_.max_measurement_age_s;
    record(m, -1, too_old ? AssociationOutcome::TooOld : AssociationOutcome::Unassigned);
  }
}

namespace {

/// What a measurement says about what the target is.
void absorb_attributes(Track& track, const Measurement& m, std::uint64_t seq, const TrackerConfig& cfg) {
  switch (m.sensor->type()) {
    case SensorType::Radar: {
      track.last_radar_hit_seq[m.sensor->id()] = seq;
      track.last_radar_update_s = std::max(track.last_radar_update_s, m.t_s);
      const auto& radar = static_cast<const RadarModel&>(*m.sensor);
      track.rcs_sum_m2 += radar.rcs_sample_m2(m.snr_db, m.z[0]);
      ++track.rcs_samples;
      break;
    }
    case SensorType::Camera:
      if (!m.label.empty()) track.class_votes[m.label] += m.confidence;
      break;
    case SensorType::RfBearing:
      if (!m.signature.empty()) {
        ++track.emitter_counts[m.signature];
        track.class_votes["drone"] += cfg.class_vote_rf_signature;  // it carries a drone control link
      }
      break;
    case SensorType::Cooperative:
      if (track.identity.empty()) track.identity = m.identity;
      ++track.cooperative_updates;
      track.class_votes[m.identity.starts_with("adsb:") ? "aircraft" : "drone"] += cfg.class_vote_broadcast;
      break;
  }
  auto [it, inserted] = track.last_update_by_sensor.try_emplace(m.sensor->id(), m.t_s);
  if (!inserted) it->second = std::max(it->second, m.t_s);
}

}  // namespace

void Tracker::update(Track& track, const Measurement& m, const Innovation& in, std::uint64_t seq) {
  // Kalman gain K = Cov(e, -nu) S^-1 = (P H' - Cz) S^-1, where Cz is the
  // covariance between the track's error and the measurement's GNSS error
  // (zero for sensors that do not share it). The error after the update is
  // (I - K H) e + K (G b + w); its covariance below holds for any gain, as
  // the Joseph form does, and stays symmetric and positive.
  const Mat63 Cq = cross_of(in.cross, in.consider);
  const Eigen::MatrixXd PHt = in.prior.P * in.H.transpose() - Cq * in.G.transpose();
  const Eigen::MatrixXd K = in.S.ldlt().solve(PHt.transpose()).transpose();
  const Mat6 IKH = Mat6::Identity() - K * in.H;
  const Mat63 KG = K * in.G;
  const Mat6 shared = IKH * Cq * KG.transpose();
  track.state.x = in.prior.x + K * in.nu;
  track.state.P = IKH * in.prior.P * IKH.transpose() + K * in.R * K.transpose() + KG * in.sigma * KG.transpose() +
                  shared + shared.transpose();
  track.state.P = 0.5 * (track.state.P + track.state.P.transpose()).eval();
  {
    // Same reason: keep P a covariance if the approximation pushed it over the edge.
    const Eigen::SelfAdjointEigenSolver<Mat6> eig(track.state.P);
    if (eig.eigenvalues().minCoeff() < kMinVariance) {
      track.state.P = eig.eigenvectors() * eig.eigenvalues().cwiseMax(kMinVariance).asDiagonal() *
                      eig.eigenvectors().transpose();
      ++stats_.covariance_repairs;
    }
  }
  // The error is now (I - K H) e + K G q + K w. The consider parameters are
  // taken as independent of each other, so Cov(e', p) = (I - K H) Cov(e, p),
  // plus K G Cov(q) for p = q.
  track.consider = in.cross;
  for (auto& [key, C] : track.consider) C = IKH * C;
  if (!in.consider.empty()) track.consider[in.consider] = cross_of(track.consider, in.consider) + KG * in.sigma;
  track.t_s = in.t_ref;
  track.last_update_s = std::max(track.last_update_s, m.t_s);
  ++track.updates;
  ++stats_.updates;
  if (in.late) ++stats_.late_updates;

  absorb_attributes(track, m, seq, cfg_);
  if (track.status == TrackStatus::Tentative && track.cooperative_updates >= cfg_.cooperative_confirm_hits) {
    confirm(track, m.t_s);
  }
  if (m.sensor->type() == SensorType::Cooperative) {
    track.broadcast = Gaussian{m.z, m.sensor->noise(m)};
    track.broadcast_t_s = m.t_s;
    track.broadcast_bias = in.sigma;
  } else if (track.broadcast && std::abs(m.t_s - track.broadcast_t_s) <= cfg_.registration_reference_max_age_s) {
    // A sensor saw a target that says where it is: evidence about the sensor's mount.
    const Gaussian reference = shifted(*track.broadcast, m.t_s - track.broadcast_t_s, cfg_.process_noise);
    if (const auto step = registration_.observe(m, reference, track.broadcast_bias, track.identity)) {
      ++stats_.registration_samples;
      // The sensor's estimate error changed for every track that carries it:
      // eps' = keep eps + terms independent of the tracks' errors, so
      // Cov(e, eps') = Cov(e, eps) keep'. (Only the reference's own track
      // shares one of those terms, its GNSS error; that coupling is left out.)
      const std::string& sensor_id = m.sensor->id();
      for (Track& other : tracks_) {
        const auto it = other.consider.find(sensor_id);
        if (it != other.consider.end()) it->second = it->second * step->transpose();
      }
    }
  }
  record(m, track.id, AssociationOutcome::Updated, in.d2);
}

void Tracker::start_consider(Track& track, const Measurement& m) const {
  if (m.sensor->type() == SensorType::Cooperative) {
    // Started from the report itself, the track's error is the report's,
    // GNSS error and all: Cov(e, b) = B, and Cov(e, eps) = Cov(b, eps).
    track.broadcast_bias = static_cast<const CooperativeModel&>(*m.sensor).bias_covariance(m);
    track.consider[std::string(kGnssError)].setZero();
    track.consider[std::string(kGnssError)].topRows<3>() = track.broadcast_bias;
    return;
  }
  if (!cfg_.consider_registration || m.registration_covariance.isZero() || m.z.size() < 3) return;
  // A position computed from a radar plot is off by (dz/dx)^-1 (v - J eps):
  // its noise, and the mount and clock error eps left after correction,
  // which every later plot of this radar shares.
  const Eigen::MatrixXd H = measurement_jacobian(*m.sensor, track.state.x, m);
  const Eigen::MatrixXd J = registration_jacobian(*m.sensor, track.state.x, m);
  if (H.size() == 0 || J.size() == 0) return;
  const Mat3 H_pos = H.topLeftCorner<3, 3>();
  if (std::abs(H_pos.determinant()) < 1e-12) return;
  const Mat3 H_inv = H_pos.inverse();
  const Eigen::Matrix3d J3 = J.topRows<3>();
  const Eigen::Matrix3d R3 = m.sensor->noise(m).topLeftCorner<3, 3>();
  track.state.P.topLeftCorner<3, 3>() =
      H_inv * (R3 + J3 * m.registration_covariance * J3.transpose()) * H_inv.transpose();
  track.state.P.topRightCorner<3, 3>().setZero();
  track.state.P.bottomLeftCorner<3, 3>().setZero();
  Mat63 C = Mat63::Zero();
  C.topRows<3>() = -H_inv * J3 * m.registration_covariance;
  track.consider[m.sensor->id()] = C;
}

void Tracker::start_track(const Measurement& m, const Gaussian& g, std::uint64_t seq) {
  Track track;
  track.id = next_track_id_++;
  track.state = g;
  track.t_s = m.t_s;
  start_consider(track, m);
  track.created_s = m.t_s;
  track.last_update_s = m.t_s;
  track.updates = 1;
  absorb_attributes(track, m, seq, cfg_);
  if (track.cooperative_updates >= cfg_.cooperative_confirm_hits) confirm(track, m.t_s);
  if (m.sensor->type() == SensorType::Cooperative) {
    track.broadcast = g;
    track.broadcast_t_s = m.t_s;
  }
  ++stats_.initiations;
  record(m, track.id, AssociationOutcome::Initiated);
  tracks_.push_back(std::move(track));
}

void Tracker::confirm(Track& track, double t) {
  track.status = TrackStatus::Confirmed;
  track.confirmed_s = t;
  ++stats_.tracks_confirmed;
}

void Tracker::record(const Measurement& m, int track_id, AssociationOutcome outcome, double d2) {
  if (outcome == AssociationOutcome::Unassigned) ++stats_.unassigned;
  if (outcome == AssociationOutcome::TooOld) ++stats_.too_old;
  associations_.push_back({m.meas_id, m.sensor->id(), m.t_s, track_id, outcome, d2});
}

// ---- track management ------------------------------------------------------------------

void Tracker::process_scan_end(const Event& event) {
  const auto& radar = dynamic_cast<const RadarModel&>(*event.sensor);
  const std::string& radar_id = radar.id();
  std::vector<int> deleted;
  for (Track& track : tracks_) {
    const Gaussian g = predicted(track.state, event.t_s - track.t_s, cfg_.process_noise);
    const Vec6& x = g.x;
    const auto it = track.last_radar_hit_seq.find(radar_id);
    const bool hit = it != track.last_radar_hit_seq.end() && it->second == event.seq;
    double& miss_log = track.radar_miss_log_prob[radar_id];
    // How likely this scan was to detect the track, from its own radar cross
    // section and the chance it was inside the Doppler notch.
    const double pd = radar.detection_probability(g, track.rcs_dbsm(cfg_.radar_assumed_rcs_dbsm));
    if (hit) {
      track.radar_history.push_back(true);
      miss_log = 0.0;
    } else if (pd > 0.0) {
      // M-of-N only counts scans that were likely to see it; the evidence
      // that nothing is there counts every miss, each by how surprising it is.
      if (pd >= cfg_.radar_min_expected_pd) track.radar_history.push_back(false);
      // Cap Pd: the model is only approximate, so no single miss is impossible.
      miss_log += std::log(1.0 - std::min(pd, cfg_.radar_max_assumed_pd));
      if (track.status == TrackStatus::Confirmed) {
        // Another sensor (not this radar) saw it recently: something is there.
        const bool seen_elsewhere = last_update_except(track, radar_id) >= event.t_s - cfg_.notch_evidence_s;
        const bool unlikely = miss_log < std::log(cfg_.notch_inference_probability);
        if (unlikely && seen_elsewhere && !radar_sees_along(radar, x, event.seq, track.id)) {
          if (infer_notch(track, radar, event.t_s)) miss_log = 0.0;  // explained: it hovers
        } else if (!seen_elsewhere && miss_log < std::log(cfg_.refute_probability)) {
          // Only this radar ever kept it alive, and it should have seen it: nothing is there.
          deleted.push_back(track.id);
          ++stats_.tracks_refuted;
        }
      }
    }
    while (static_cast<int>(track.radar_history.size()) > cfg_.confirm_window) track.radar_history.pop_front();

    if (track.status != TrackStatus::Tentative) continue;
    const auto hits = std::count(track.radar_history.begin(), track.radar_history.end(), true);
    if (hits >= cfg_.confirm_hits) {
      confirm(track, event.t_s);
    } else if (track.cooperative_updates == 0 && static_cast<int>(track.radar_history.size()) >= cfg_.confirm_window) {
      deleted.push_back(track.id);  // M-of-N failed: most likely clutter
    }
  }
  if (!deleted.empty()) {
    std::erase_if(tracks_, [&](const Track& t) { return std::find(deleted.begin(), deleted.end(), t.id) != deleted.end(); });
    stats_.tracks_deleted += static_cast<int>(deleted.size());
  }
}

bool Tracker::radar_sees_along(const RadarModel& radar, const Vec6& x, std::uint64_t seq, int except_track) const {
  // Did this scan detect another track in (almost) the same direction? Then
  // the radar is not blind there, and a missing detection is not the notch:
  // more likely this track is a ghost of that one.
  const Vec3 los = (x.head<3>() - radar.position()).normalized();
  for (const Track& other : tracks_) {
    if (other.id == except_track) continue;
    const auto it = other.last_radar_hit_seq.find(radar.id());
    if (it == other.last_radar_hit_seq.end() || it->second != seq) continue;
    const Vec3 other_los = (other.state.x.head<3>() - radar.position()).normalized();
    if (los.dot(other_los) > std::cos(geo::deg2rad(cfg_.radar_conflict_deg))) return true;
  }
  return false;
}

bool Tracker::infer_notch(Track& track, const RadarModel& radar, double t) {
  // The radar keeps missing a target it should see, while other sensors
  // still see it. The likeliest reason is that it stopped moving relative to
  // the radar and fell into the Doppler notch with the ground clutter. Say
  // so to the filter: range rate ~ uniform within the notch. Without this a
  // hovering drone's track keeps flying at its old speed, because angle-only
  // sensors next to the radar cannot see it is no longer closing in.
  const double notch = radar.doppler_notch_mps();
  if (!(notch > 0.0)) return false;  // this radar has no notch to hide in
  // A track that is sure the target is still closing or receding is not
  // hovering: the radar's misses are bad luck (a fading target, a low RCS),
  // and forcing its range rate to zero would push it off along the line of
  // sight. Hovering only fits a track whose range rate could be ~0.
  Gaussian g = predicted(track.state, t - track.t_s, cfg_.process_noise);
  ConsiderCross cross = carried(track.consider, t - track.t_s, cfg_.broadcast_error_correlation_s);
  const double nis = scalar_update(g, [&radar](const Vec6& x) { return radar.range_rate(x); }, 0.0,
                                   notch * notch / 3.0, cfg_.notch_gate_chi2, &cross);
  if (nis > cfg_.notch_gate_chi2) {
    ++stats_.notch_rejections;
    return false;
  }
  track.state = g;
  track.consider = std::move(cross);
  track.t_s = std::max(track.t_s, t);
  track.last_notch_s = t;
  ++stats_.notch_inferences;
  return true;
}

void Tracker::maintain(double t) {
  // Delete tracks nobody has seen for too long, or that have grown too vague to be useful.
  const auto before = tracks_.size();
  std::erase_if(tracks_, [&](const Track& track) {
    const double coast_limit =
        track.status == TrackStatus::Confirmed ? cfg_.confirmed_coast_s : cfg_.tentative_coast_s;
    if (t - track.last_update_s > coast_limit) return true;
    const Gaussian g = predicted(track.state, t - track.t_s, cfg_.process_noise);
    return std::sqrt(g.P.topLeftCorner<3, 3>().diagonal().maxCoeff()) > cfg_.max_position_std_m;
  });
  stats_.tracks_deleted += static_cast<int>(before - tracks_.size());

  if (!cfg_.merge_duplicates) return;
  remove_line_of_sight_ghosts(t);
  // Two confirmed tracks that agree in position and velocity are one target
  // tracked twice (for example, started by radar and by ADS-B). Tracks are
  // kept in order of id, so the older one is always `keep`.
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    for (std::size_t j = i + 1; j < tracks_.size();) {
      Track& keep = tracks_[i];
      const Track& drop = tracks_[j];
      const bool candidates = keep.status == TrackStatus::Confirmed && drop.status == TrackStatus::Confirmed &&
                              !identities_conflict(keep.identity, drop.identity);
      // Compare (and fuse) at a common time, never moving either back.
      const double at = std::max({t, keep.t_s, drop.t_s});
      const Gaussian gk = predicted(keep.state, at - keep.t_s, cfg_.process_noise);
      const Gaussian gd = predicted(drop.state, at - drop.t_s, cfg_.process_noise);
      const Vec6 d = gk.x - gd.x;
      if (!candidates || d.dot(difference_covariance(keep, gk, drop, gd, at).ldlt().solve(d)) > cfg_.merge_gate_chi2) {
        ++j;
        continue;
      }
      merge_into(keep, drop, gk, gd, at);
      tracks_.erase(tracks_.begin() + static_cast<std::ptrdiff_t>(j));
      ++stats_.tracks_merged;
    }
  }
}

void Tracker::merge_into(Track& keep, const Track& drop, const Gaussian& gk, const Gaussian& gd, double at) const {
  for (const auto& [cls, votes] : drop.class_votes) keep.class_votes[cls] += votes;
  for (const auto& [name, count] : drop.emitter_counts) keep.emitter_counts[name] += count;
  if (keep.identity.empty()) keep.identity = drop.identity;
  keep.cooperative_updates += drop.cooperative_updates;
  for (const auto& [sensor, when] : drop.last_update_by_sensor) {
    auto [it, inserted] = keep.last_update_by_sensor.try_emplace(sensor, when);
    if (!inserted) it->second = std::max(it->second, when);
  }
  // Both followed one target, often with the same sensors, and nobody
  // kept the cross-covariance of their errors: covariance intersection
  // fuses them consistently whatever it was.
  const double tau = cfg_.broadcast_error_correlation_s;
  const ConsiderCross ck = carried(keep.consider, at - keep.t_s, tau);
  const ConsiderCross cd = carried(drop.consider, at - drop.t_s, tau);
  const Intersection ci = covariance_intersection(gk, gd);
  keep.state = ci.fused;
  keep.t_s = at;
  keep.consider.clear();
  for (const ConsiderCross* c : {&ck, &cd}) {
    for (const auto& [key, C] : *c) {
      keep.consider[key] = ci.gain_a * cross_of(ck, key) + ci.gain_b * cross_of(cd, key);
    }
  }
  if (keep.broadcast_bias.isZero()) keep.broadcast_bias = drop.broadcast_bias;
  if (drop.broadcast && (!keep.broadcast || drop.broadcast_t_s > keep.broadcast_t_s)) {
    keep.broadcast = drop.broadcast;
    keep.broadcast_t_s = drop.broadcast_t_s;
  }
  // What either knows about the radars: latest hit, fewest unexplained misses, all RCS samples.
  for (const auto& [radar, seq] : drop.last_radar_hit_seq) {
    auto [it, inserted] = keep.last_radar_hit_seq.try_emplace(radar, seq);
    if (!inserted) it->second = std::max(it->second, seq);
  }
  for (const auto& [radar, log_p] : drop.radar_miss_log_prob) {
    auto [it, inserted] = keep.radar_miss_log_prob.try_emplace(radar, log_p);
    if (!inserted) it->second = std::max(it->second, log_p);
  }
  keep.rcs_sum_m2 += drop.rcs_sum_m2;
  keep.rcs_samples += drop.rcs_samples;
  keep.last_radar_update_s = std::max(keep.last_radar_update_s, drop.last_radar_update_s);
  keep.last_notch_s = std::max(keep.last_notch_s, drop.last_notch_s);
  keep.last_update_s = std::max(keep.last_update_s, drop.last_update_s);
}

Mat6 Tracker::difference_covariance(const Track& a, const Gaussian& ga, const Track& b, const Gaussian& gb,
                                    double t) const {
  // Two tracks fed by the same sensors share those sensors' registration
  // errors, so their difference is better known than P_a + P_b suggests:
  // Cov(e_a - e_b) = P_a + P_b - Cov(e_a, e_b) - Cov(e_b, e_a), where the
  // shared part runs through each common consider parameter p:
  // Cov(e_a, e_b) ~ sum_p Cov(e_a, p) Cov(p)^-1 Cov(p, e_b).
  const ConsiderCross ca = carried(a.consider, t - a.t_s, cfg_.broadcast_error_correlation_s);
  const ConsiderCross cb = carried(b.consider, t - b.t_s, cfg_.broadcast_error_correlation_s);
  Mat6 shared = Mat6::Zero();
  for (const auto& [key, Ca] : ca) {
    const auto it = cb.find(key);
    if (it == cb.end() || key == kGnssError) continue;  // two targets' GNSS errors are independent
    const Eigen::Matrix3d sigma = registration_.estimate(key).P;
    const auto ldlt = sigma.ldlt();
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) continue;
    shared += Ca * ldlt.solve(it->second.transpose());
  }
  const Mat6 full = ga.P + gb.P - shared - shared.transpose();
  // The approximation must not make the two tracks look more alike than either is certain.
  const auto check = full.ldlt();
  return check.info() == Eigen::Success && check.isPositive() ? full : Mat6(ga.P + gb.P);
}

void Tracker::remove_line_of_sight_ghosts(double t) {
  // Sensors at one site that measure only angles (cameras, direction
  // finders) cannot tell two tracks on the same line of sight apart, so they
  // can keep a stale copy of a target alive at the wrong range. If one of two
  // such tracks still gets radar updates and the other has not for a while,
  // the other is the ghost: drop it.
  std::vector<int> ghosts;
  for (const Track& live : tracks_) {
    if (live.status != TrackStatus::Confirmed || t - live.last_radar_update_s > cfg_.ghost_live_radar_s) continue;
    const Vec3 live_los = ncv_transition(t - live.t_s).topRows<3>() * live.state.x;
    for (Track& ghost : tracks_) {
      if (&ghost == &live || ghost.status != TrackStatus::Confirmed) continue;
      if (t - ghost.last_radar_update_s < cfg_.ghost_stale_radar_s || !ghost.identity.empty()) continue;
      // A track the radar misses because it hovers in the Doppler notch is
      // not a ghost, even when another target crosses its line of sight.
      if (t - ghost.last_notch_s < cfg_.ghost_stale_radar_s) continue;
      const Vec3 ghost_los = ncv_transition(t - ghost.t_s).topRows<3>() * ghost.state.x;
      if (live_los.normalized().dot(ghost_los.normalized()) < std::cos(geo::deg2rad(cfg_.ghost_direction_deg))) continue;
      if (std::find(ghosts.begin(), ghosts.end(), ghost.id) != ghosts.end()) continue;
      ghosts.push_back(ghost.id);
    }
  }
  if (ghosts.empty()) return;
  std::erase_if(tracks_, [&](const Track& x) { return std::find(ghosts.begin(), ghosts.end(), x.id) != ghosts.end(); });
  stats_.ghosts_removed += static_cast<int>(ghosts.size());
}

// ---- output -----------------------------------------------------------------------------

void Tracker::publish_before(double t, bool inclusive) {
  if (!next_output_index_) next_output_index_ = std::ceil(t / output_period_s_ - 1e-9);
  // Output times are index * period, so they never accumulate rounding.
  while (true) {
    const double to = *next_output_index_ * output_period_s_;
    if (inclusive ? to > t : to >= t) break;
    maintain(to);
    for (const Track& track : tracks_) {
      TrackReport r;
      r.t_s = to;
      r.track_id = track.id;
      r.status = track.status;
      r.classification = track.classification();
      r.identity = track.identity;
      r.emitter = track.emitter();
      r.state = predicted(track.state, to - track.t_s, cfg_.process_noise);
      r.last_update_s = track.last_update_s;
      r.t_published_s = latest_arrival_s_;
      for (const auto& [sensor, when] : track.last_update_by_sensor) {
        if (to - when > kRecentSensorsS) continue;
        if (!r.sensors.empty()) r.sensors += '+';
        r.sensors += sensor;
      }
      reports_.push_back(std::move(r));
    }
    if (!first_output_s_) first_output_s_ = to;
    last_output_s_ = to;
    *next_output_index_ += 1.0;
  }
}

std::optional<std::pair<double, double>> Tracker::output_span() const {
  if (!first_output_s_) return std::nullopt;
  return std::make_pair(*first_output_s_, last_output_s_);
}

}  // namespace skytrack
