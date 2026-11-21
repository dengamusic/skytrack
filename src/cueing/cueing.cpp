#include "skytrack/cueing/cueing.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "skytrack/run_writer.hpp"
#include "skytrack/sensors/camera.hpp"
#include "skytrack/tracking/sensor_model.hpp"

namespace skytrack {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

Vec3 position_at(const TrackReport& r, double t) {
  return r.state.x.head<3>() + (t - r.t_s) * r.state.x.tail<3>();
}

bool classified(const TrackReport& r) { return !r.classification.empty() && r.classification != "unknown"; }

const TrackReport* find_track(const std::vector<TrackReport>& picture, std::optional<int> id) {
  if (!id) return nullptr;
  for (const TrackReport& r : picture) {
    if (r.track_id == *id && r.status == TrackStatus::Confirmed) return &r;
  }
  return nullptr;
}

}  // namespace

CueController::CueController(CueConfig config, const Mount& mount, GimbalReport start)
    : cfg_(std::move(config)), mount_(mount), pointing_(start) {}

GimbalReport CueController::pointing_to(const Vec3& p_enu) const {
  // Body axes: forward, left, up. Pan is clockwise, tilt is up (see Camera).
  const Vec3 b = mount_.R_world_from_body().transpose() * (p_enu - mount_.position_enu);
  return {std::atan2(-b.y(), b.x()), std::atan2(b.z(), std::hypot(b.x(), b.y()))};
}

std::optional<int> CueController::choose(double t, const std::vector<TrackReport>& picture) const {
  const double slew_rad_s = geo::deg2rad(cfg_.max_slew_deg_s);
  // Lexicographic: (rank, key), smaller first.
  std::optional<int> best;
  std::pair<int, double> best_score{std::numeric_limits<int>::max(), kInf};
  for (const TrackReport& r : picture) {
    if (r.status != TrackStatus::Confirmed || !r.identity.empty()) continue;
    const auto seen = last_look_.find(r.track_id);
    const double unseen_s = seen == last_look_.end() ? kInf : t - seen->second;

    const Vec3 p = position_at(r, t);
    const Vec3 to_target = p - mount_.position_enu;
    const double closing_mps = std::max(1.0, -to_target.normalized().dot(r.state.x.tail<3>()));
    const GimbalReport aim = pointing_to(p);
    const double slew_s = std::max(std::abs(geo::wrap_pi(aim.pan_rad - pointing_.pan_rad)),
                                   std::abs(aim.tilt_rad - pointing_.tilt_rad)) /
                          slew_rad_s;
    const double urgency_s = to_target.norm() / closing_mps + slew_s;

    std::pair<int, double> score;
    if (!classified(r) && unseen_s == kInf) {
      score = {0, urgency_s};  // never looked at, no class: the camera's main job
    } else if (!classified(r) && unseen_s > cfg_.revisit_s / 3.0) {
      score = {1, urgency_s};  // looked at without result: try again, less often
    } else if (r.classification != "drone" && unseen_s > cfg_.revisit_s) {
      score = {2, -unseen_s};  // check the class by eye now and then, oldest first
    } else if (r.classification == "drone") {
      score = {3, urgency_s};  // custody of the threat
    } else {
      continue;
    }
    if (score < best_score) {
      best_score = score;
      best = r.track_id;
    }
  }
  return best;
}

GimbalReport CueController::step(double t, const std::vector<TrackReport>& picture) {
  const double dt = started_ ? std::max(0.0, t - last_t_) : 0.0;
  started_ = true;
  last_t_ = t;
  ++stats_.frames;

  const TrackReport* current = find_track(picture, target_);
  bool keep = false;
  if (current != nullptr) {
    if (!on_target_since_) {
      keep = t - chosen_at_ < cfg_.max_slew_time_s;
      if (!keep) ++stats_.gave_up;
    } else {
      const double on_s = t - *on_target_since_;
      keep = on_s < cfg_.min_dwell_s || (!classified(*current) && on_s < cfg_.max_dwell_s);
    }
  }
  if (!keep) {
    if (target_) last_look_[*target_] = t;
    const std::optional<int> next = choose(t, picture);
    if (next != target_ || !on_target_since_) {
      target_ = next;
      chosen_at_ = t;
      on_target_since_.reset();
      if (target_) ++stats_.looks;
    } else {
      on_target_since_ = t;  // staying on the same track (custody): a new dwell
    }
    current = find_track(picture, target_);
  }

  if (current != nullptr) {
    const GimbalReport aim = pointing_to(position_at(*current, t));
    const double max_step = geo::deg2rad(cfg_.max_slew_deg_s) * dt;
    pointing_.pan_rad =
        geo::wrap_pi(pointing_.pan_rad + std::clamp(geo::wrap_pi(aim.pan_rad - pointing_.pan_rad), -max_step, max_step));
    pointing_.tilt_rad += std::clamp(aim.tilt_rad - pointing_.tilt_rad, -max_step, max_step);
    const double error = std::max(std::abs(geo::wrap_pi(aim.pan_rad - pointing_.pan_rad)),
                                  std::abs(aim.tilt_rad - pointing_.tilt_rad));
    if (error <= geo::deg2rad(cfg_.on_target_deg)) {
      ++stats_.frames_on_target;
      if (!on_target_since_) on_target_since_ = t;
    }
  }
  return pointing_;
}

CuedRun run_cued(const Scenario& scenario, const TrackerConfig& tracker_config, const CueConfig& cue_config) {
  const Camera* camera = nullptr;
  std::size_t camera_index = 0;
  for (std::size_t i = 0; i < scenario.sensors.size(); ++i) {
    if (scenario.sensors[i]->id() == cue_config.camera_id) {
      camera = dynamic_cast<const Camera*>(scenario.sensors[i].get());
      camera_index = i;
    }
  }
  if (camera == nullptr || !camera->has_gimbal()) {
    throw std::invalid_argument("no gimbal camera '" + cue_config.camera_id + "' in the scenario");
  }

  SimulationResult sim = simulate(scenario, {cue_config.camera_id});
  auto setup = std::make_shared<const SensorSetup>(nominal_sensors_json(scenario), tracker_config);
  Tracker tracker(setup, tracker_config);
  CueController controller(cue_config, camera->common().mount, camera->gimbal_at(0.0));
  const World world{scenario.targets, scenario.frame};
  Rng detections = detection_rng(scenario, *camera);
  Rng network = network_rng(scenario, *camera);
  SensorStats stats;
  stats.sensor_id = camera->id();
  stats.type = camera->type();
  double last_arrival = -kInf;

  // Deliver messages to the tracker in arrival order: the other sensors'
  // (all known in advance) and the camera's (made frame by frame).
  std::vector<SensorMessage> delivered;
  std::deque<SensorMessage> in_flight;
  std::size_t next = 0;
  std::int64_t next_meas_id = 0;
  const auto deliver_until = [&](double t) {
    for (;;) {
      const bool other = next < sim.messages.size() && sim.messages[next].t_arrival_s <= t;
      const bool mine = !in_flight.empty() && in_flight.front().t_arrival_s <= t;
      if (!other && !mine) return;
      SensorMessage msg;
      if (other && (!mine || sim.messages[next].t_arrival_s <= in_flight.front().t_arrival_s)) {
        msg = std::move(sim.messages[next++]);
      } else {
        msg = std::move(in_flight.front());
        in_flight.pop_front();
      }
      for (Detection& d : msg.detections) d.meas_id = next_meas_id++;
      tracker.ingest(msg);
      delivered.push_back(std::move(msg));
    }
  };

  std::vector<TrackReport> picture;
  for (std::uint64_t k = 0;; ++k) {
    const double t = camera->common().phase_s + static_cast<double>(k) * camera->period_s();
    if (t > scenario.duration_s) break;
    deliver_until(t);
    picture.clear();
    const auto& reports = tracker.reports();
    for (auto it = reports.rbegin(); it != reports.rend() && it->t_s == reports.back().t_s; ++it) {
      picture.push_back(*it);
    }
    const GimbalReport pointing = controller.step(t, picture);
    Scan scan = camera->observe_pointing(t, world, detections, pointing);
    ++stats.scans;
    const bool dropped = network.bernoulli(camera->common().errors.dropout_prob);
    const double latency = std::max(
        0.0, network.normal(camera->common().errors.latency_mean_s, camera->common().errors.latency_jitter_s));
    if (dropped) {
      ++stats.dropped;
      continue;
    }
    in_flight.push_back(make_message(*camera, std::move(scan), k, latency, last_arrival, stats));
  }
  deliver_until(kInf);
  tracker.finish();

  sim.messages = std::move(delivered);
  sim.stats.insert(sim.stats.begin() + static_cast<std::ptrdiff_t>(std::min(camera_index, sim.stats.size())),
                   std::move(stats));
  return CuedRun{std::move(sim), std::move(tracker), controller.stats()};
}

}  // namespace skytrack
