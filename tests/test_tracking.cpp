#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "skytrack/eval/metrics.hpp"
#include "skytrack/rng.hpp"
#include "skytrack/run_writer.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/sensors/radar.hpp"
#include "skytrack/simulator.hpp"
#include "skytrack/tracking/config.hpp"
#include "skytrack/tracking/sensor_model.hpp"
#include "skytrack/tracking/track_io.hpp"
#include "skytrack/tracking/tracker.hpp"
#include "test_helpers.hpp"

namespace skytrack {
namespace {

std::filesystem::path scenario_path(const std::string& name) {
  return std::filesystem::path(SKYTRACK_SOURCE_DIR) / "scenarios" / (name + ".json");
}

/// A tracker run over a simulated scenario, scored against its truth.
struct Outcome {
  std::shared_ptr<const SensorSetup> setup;
  std::unique_ptr<Tracker> tracker;
  eval::Evaluation evaluation;
};

Outcome track(const Scenario& scenario, const SimulationResult& sim, const TrackerConfig& config) {
  Outcome o;
  o.setup = std::make_shared<const SensorSetup>(nominal_sensors_json(scenario), config);
  o.tracker = std::make_unique<Tracker>(run_tracker(o.setup, config, sim.messages));
  if (const auto span = o.tracker->output_span()) {
    o.evaluation = eval::evaluate(eval::truth_from_scenario(scenario), o.tracker->reports(), span->first,
                                  std::min(span->second, scenario.duration_s), config.output_rate_hz,
                                  eval::EvalConfig{});
  }
  return o;
}

const eval::TargetScore& score(const Outcome& o, std::string_view name) {
  const auto& targets = o.evaluation.targets;
  const auto it = std::find_if(targets.begin(), targets.end(), [&](const auto& t) { return t.name == name; });
  if (it == targets.end()) throw std::runtime_error("no target named " + std::string(name));
  return *it;
}

/// The baseline C-UAS scene (seed 42), simulated once for all tests that use it.
const Scenario& baseline() {
  static const Scenario scenario = load_scenario(scenario_path("cuas_baseline"));
  return scenario;
}
const SimulationResult& baseline_sim() {
  static const SimulationResult sim = simulate(baseline());
  return sim;
}
const Outcome& baseline_outcome() {
  static const Outcome outcome = track(baseline(), baseline_sim(), TrackerConfig{});
  return outcome;
}

// ---- sensor models ----------------------------------------------------------

/// The tracker's measurement functions h(x) must reproduce what the simulator
/// reports when every error is switched off: same frames, same angle
/// conventions, same gimbal and lens model, same geodesy. Every other test
/// relies on this.
TEST(SensorModels, PredictMatchesNoiseFreeSimulator) {
  Json j = read_json_file(scenario_path("cuas_baseline"));
  for (Json& target : j["targets"]) {
    if (target.contains("cooperative")) {
      target["cooperative"]["horizontal_std_m"] = 0.0;
      target["cooperative"]["vertical_std_m"] = 0.0;
    }
  }
  for (Json& sensor : j["sensors"]) {
    sensor["errors"] = Json::object();
    sensor["errors"]["noise_scale"] = 0.0;
    const std::string type = sensor["type"];
    if (type == "radar") sensor["radar"]["false_alarms_per_scan"] = 0.0;
    if (type == "camera") sensor["camera"]["false_alarms_per_frame"] = 0.0;
    if (type == "rf_bearing") sensor["rf_bearing"]["false_alarms_per_report"] = 0.0;
    if (type == "cooperative") sensor["cooperative"]["velocity_std_mps"] = 0.0;
    if (sensor["id"] == "cam-wide") {  // exercise the lens distortion too
      sensor["camera"]["k1"] = -0.12;
      sensor["camera"]["k2"] = 0.03;
    }
  }
  const Scenario scenario = parse_scenario(j);
  const SimulationResult sim = simulate(scenario);
  const TrackerConfig config;
  const SensorSetup setup(nominal_sensors_json(scenario), config);

  std::map<SensorType, int> checked;
  for (const SensorMessage& msg : sim.messages) {
    const SensorModel* model = setup.find(msg.sensor_id);
    ASSERT_NE(model, nullptr) << msg.sensor_id;
    for (const Detection& det : msg.detections) {
      ASSERT_GE(det.origin_target_id, 0) << "false alarms were switched off";
      const Measurement m = model->convert(det, msg);
      ASSERT_NEAR(m.t_s, det.t_true_s, 1e-9) << msg.sensor_id;

      const auto target = std::find_if(scenario.targets.begin(), scenario.targets.end(),
                                       [&](const Target& t) { return t.id == det.origin_target_id; });
      const KinematicState truth = target->trajectory.state(det.t_true_s);
      Vec6 x;
      x << truth.position, truth.velocity;
      const auto zhat = model->predict(x, m);
      ASSERT_TRUE(zhat.has_value()) << msg.sensor_id << " cannot see a target it detected";
      const Eigen::VectorXd r = model->residual(m.z, *zhat);

      switch (model->type()) {
        case SensorType::Radar:
          EXPECT_NEAR(r[0], 0.0, 1e-6) << "range";
          EXPECT_NEAR(r[1], 0.0, 1e-9) << "azimuth";
          EXPECT_NEAR(r[2], 0.0, 1e-9) << "elevation";
          EXPECT_NEAR(r[3], 0.0, 1e-9) << "range rate";
          break;
        case SensorType::Camera:
          EXPECT_NEAR(r[0], 0.0, 1e-6) << msg.sensor_id << " u";
          EXPECT_NEAR(r[1], 0.0, 1e-6) << msg.sensor_id << " v";
          break;
        case SensorType::RfBearing:
          EXPECT_NEAR(r[0], 0.0, 1e-9) << "bearing";
          break;
        case SensorType::Cooperative:
          EXPECT_LT(r.head<3>().cwiseAbs().maxCoeff(), 1e-3) << "position through WGS84 and back";
          EXPECT_LT(r.tail<3>().cwiseAbs().maxCoeff(), 1e-9) << "velocity";
          break;
      }
      ++checked[model->type()];
    }
  }
  for (const SensorType type : {SensorType::Radar, SensorType::Camera, SensorType::RfBearing, SensorType::Cooperative}) {
    EXPECT_GT(checked[type], 50) << to_string(type);
  }
}

/// A radar only reports targets whose echo crossed its threshold, so the SNR
/// of the detections it does report overstates the target's mean echo. The
/// tracker's RCS samples must undo that, or it would think small, distant
/// drones are bigger than they are and expect to see them when it cannot.
TEST(SensorModels, RadarCrossSectionSamplesAreUnbiased) {
  Json j = Json::parse(R"({"id": "radar", "type": "radar", "rate_hz": 1,
                           "radar": {"mode": "staring", "fov_az_deg": 90, "max_range_m": 8000, "snr_ref_db": 53}})");
  const auto sensor = parse_sensor(j);
  const auto& radar = dynamic_cast<const Radar&>(*sensor);
  const RadarModel model(radar, SensorOverrides{});

  const double rcs_dbsm = -20.0;
  const double range_m = 4500.0;  // weak: only about one scan in ten detects it
  const std::vector<Target> targets{test::straight_target(1, Vec3(0, range_m, 0), Vec3(0, 10, 0),
                                                          TargetClass::Drone, rcs_dbsm)};
  const geo::LocalFrame frame = parse_origin(Json::parse(R"({"lat_deg": 48, "lon_deg": 11})"));
  const World world{targets, frame};

  Rng rng(3);
  double sum = 0.0;
  double naive_sum = 0.0;
  int n = 0;
  for (int scan = 0; scan < 60000; ++scan) {
    for (const Detection& det : radar.observe(0.0, world, rng).detections) {
      if (det.origin_target_id != 1) continue;  // false alarms have no cross section
      const auto& d = std::get<RadarDetection>(det.payload);
      sum += model.rcs_sample_m2(d.snr_db, d.range_m);
      naive_sum += std::pow(10.0, (d.snr_db - radar.snr_db(d.range_m, 0.0)) / 10.0);  // SNR taken at face value
      ++n;
    }
  }
  ASSERT_GT(n, 3000);
  const double truth_m2 = std::pow(10.0, rcs_dbsm / 10.0);
  EXPECT_NEAR(sum / n / truth_m2, 1.0, 0.05);
  EXPECT_GT(naive_sum / n / truth_m2, 2.0) << "the threshold bias this corrects is large";
}

// ---- configuration ----------------------------------------------------------

TEST(TrackerConfig, ParsingIsStrictAndRoundTrips) {
  EXPECT_ANY_THROW(parse_tracker_config(Json::parse(R"({"reorder_windw_s": 1.0})")));
  EXPECT_ANY_THROW(parse_tracker_config(Json::parse(R"({"confirm_hits": "two"})")));
  EXPECT_ANY_THROW(parse_tracker_config(Json::parse(R"({"sensors": {"radar-1": {"clock_ofset_s": 0.1}}})")));
  EXPECT_ANY_THROW(parse_tracker_config(Json::parse(R"({"confirm_hits": 4, "confirm_window": 3})")));

  TrackerConfig config = parse_tracker_config(Json::parse(R"({
      "confirm_hits": 3, "confirm_window": 4, "registration": false,
      "process_noise": {"horizontal_m2_s3": 2.0, "vertical_m2_s3": 0.25},
      "sensors": {"rf-df-1": {"assumed_latency_s": 0.3}}})"));
  EXPECT_EQ(config.confirm_hits, 3);
  EXPECT_FALSE(config.registration);
  EXPECT_DOUBLE_EQ(config.process_noise.vertical_m2_s3, 0.25);
  EXPECT_DOUBLE_EQ(config.sensors.at("rf-df-1").assumed_latency_s, 0.3);
  EXPECT_EQ(to_json(parse_tracker_config(to_json(config))), to_json(config));

  // A sensor the site does not have is a typo, not something to ignore.
  config.sensors["radar-2"] = SensorOverrides{};
  EXPECT_ANY_THROW(SensorSetup(nominal_sensors_json(baseline()), config));
}

/// configs/default.json documents every setting: it must list exactly the
/// built-in defaults, so a new setting or a changed default updates it too.
TEST(TrackerConfig, DefaultFileMatchesTheBuiltInDefaults) {
  const Json file = read_json_file(std::filesystem::path(SKYTRACK_SOURCE_DIR) / "configs" / "default.json");
  EXPECT_EQ(file, to_json(TrackerConfig{}));
}

// ---- end to end -------------------------------------------------------------

TEST(Tracker, FollowsASingleDroneWithOneHonestTrack) {
  const Scenario scenario = load_scenario(scenario_path("single_drone"));
  const Outcome o = track(scenario, simulate(scenario), TrackerConfig{});
  ASSERT_EQ(o.evaluation.targets.size(), 1u);
  const eval::TargetScore& drone = o.evaluation.targets.front();
  EXPECT_GT(drone.tracked_fraction, 0.9);
  EXPECT_EQ(drone.track_ids.size(), 1u);
  EXPECT_EQ(drone.gaps, 0);
  EXPECT_LT(drone.rmse_horizontal_m, 10.0);
  // Position NEES averages 3 for an honest covariance; well above means overconfident.
  EXPECT_GT(drone.nees_mean, 0.5);
  EXPECT_LT(drone.nees_mean, 6.0);
  EXPECT_EQ(o.evaluation.false_track_count(), 0);
}

/// The single-drone scene, simulated once.
const Scenario& single_drone() {
  static const Scenario scenario = load_scenario(scenario_path("single_drone"));
  return scenario;
}
const SimulationResult& single_drone_sim() {
  static const SimulationResult sim = simulate(single_drone());
  return sim;
}

/// The simulation with its messages edited, in arrival order.
template <typename Edit>
SimulationResult edited(const SimulationResult& sim, Edit edit) {
  SimulationResult out = sim;
  edit(out.messages);
  std::stable_sort(out.messages.begin(), out.messages.end(),
                   [](const SensorMessage& a, const SensorMessage& b) { return a.t_arrival_s < b.t_arrival_s; });
  return out;
}

/// Radar scans that arrive after the reorder window has closed on them are
/// still used: the track is retrodicted to the scan time, updated with the
/// extra uncertainty of predicting backwards, and carried forward again.
TEST(Tracker, UsesRadarScansThatArriveAfterTheReorderWindow) {
  const TrackerConfig config;
  const auto is_delayed = [](const SensorMessage& m) { return m.sensor_id == "radar-1" && m.seq % 3 == 1; };
  const SimulationResult late = edited(single_drone_sim(), [&](std::vector<SensorMessage>& messages) {
    for (SensorMessage& m : messages) {
      if (is_delayed(m)) m.t_arrival_s += config.reorder_window_s + 1.5;
    }
  });
  const SimulationResult lost = edited(single_drone_sim(), [&](std::vector<SensorMessage>& messages) {
    std::erase_if(messages, is_delayed);
  });

  const Outcome on_time = track(single_drone(), single_drone_sim(), config);
  const Outcome with_late = track(single_drone(), late, config);
  const Outcome without = track(single_drone(), lost, config);
  EXPECT_EQ(on_time.tracker->stats().late_updates, 0u);
  EXPECT_GT(with_late.tracker->stats().late_updates, 10u);

  const eval::TargetScore& a = on_time.evaluation.targets.front();
  const eval::TargetScore& b = with_late.evaluation.targets.front();
  const eval::TargetScore& c = without.evaluation.targets.front();
  EXPECT_EQ(b.track_ids.size(), 1u);
  EXPECT_LT(b.rmse_horizontal_m, 1.25 * a.rmse_horizontal_m);
  EXPECT_LT(b.rmse_horizontal_m, c.rmse_horizontal_m);  // late data still helps
  EXPECT_GT(b.nees_mean, 0.5);
  EXPECT_LT(b.nees_mean, 6.0);
}

/// Detections of the drone removed from the given sensors after t0 (and before t1).
SimulationResult hide_drone(const std::vector<std::string>& sensors, double t0, double t1) {
  return edited(single_drone_sim(), [&](std::vector<SensorMessage>& messages) {
    for (SensorMessage& m : messages) {
      if (std::find(sensors.begin(), sensors.end(), m.sensor_id) == sensors.end()) continue;
      std::erase_if(m.detections, [&](const Detection& d) {
        return d.origin_target_id >= 0 && d.t_true_s >= t0 && d.t_true_s < t1;
      });
    }
  });
}

/// When nothing sees a confirmed track any more and the radar should have,
/// it is deleted after a few scans instead of coasting for confirmed_coast_s.
TEST(Tracker, RefutesATrackThatNoSensorSeesAnyMore) {
  const TrackerConfig config;
  const double vanish_s = 30.0;
  const Outcome o = track(single_drone(), hide_drone({"radar-1", "cam-1"}, vanish_s, 1e9), config);
  EXPECT_GE(o.tracker->stats().tracks_refuted, 1);
  double last_confirmed = -1.0;
  for (const TrackReport& r : o.tracker->reports()) {
    if (r.status == TrackStatus::Confirmed) last_confirmed = std::max(last_confirmed, r.t_s);
  }
  EXPECT_GT(last_confirmed, vanish_s - 1.0);
  EXPECT_LT(last_confirmed, vanish_s + 0.75 * config.confirmed_coast_s);
}

/// Radar misses alone do not refute a track that the camera still sees: the
/// radar may be blind there (a fade, the Doppler notch). Ten seconds without
/// radar keep one track on the drone.
TEST(Tracker, DoesNotRefuteATrackAnotherSensorStillSees) {
  const SimulationResult radar_gap = hide_drone({"radar-1"}, 30.0, 40.0);
  const Outcome o = track(single_drone(), radar_gap, TrackerConfig{});
  EXPECT_EQ(o.tracker->stats().tracks_refuted, 0);
  const eval::TargetScore& drone = o.evaluation.targets.front();
  EXPECT_EQ(drone.track_ids.size(), 1u);
  EXPECT_EQ(drone.gaps, 0);
  EXPECT_GT(drone.tracked_fraction, 0.9);

  // The same gap, if the camera's support did not count, refutes the track.
  TrackerConfig radar_only;
  radar_only.notch_evidence_s = 0.0;
  const Outcome blind = track(single_drone(), radar_gap, radar_only);
  EXPECT_GE(blind.tracker->stats().tracks_refuted, 1);
  EXPECT_GT(blind.evaluation.targets.front().track_ids.size(), 1u);
}

/// A split plot: for its first seconds the radar reports the drone twice, so
/// two tracks start on it and both confirm. They are merged by covariance
/// intersection, and the merged track stays honest.
TEST(Tracker, MergesTwoTracksStartedOnOneTarget) {
  const SimulationResult split = edited(single_drone_sim(), [](std::vector<SensorMessage>& messages) {
    for (SensorMessage& m : messages) {
      if (m.sensor_id != "radar-1") continue;
      std::vector<Detection> copies;
      for (const Detection& d : m.detections) {
        if (d.origin_target_id >= 0 && d.t_true_s < 4.0) copies.push_back(d);
      }
      m.detections.insert(m.detections.end(), copies.begin(), copies.end());
    }
  });
  const TrackerConfig config;
  const Outcome clean = track(single_drone(), single_drone_sim(), config);
  const Outcome twice = track(single_drone(), split, config);
  EXPECT_EQ(clean.tracker->stats().tracks_merged, 0);
  EXPECT_GE(twice.tracker->stats().tracks_merged, 1);

  const eval::TargetScore& a = clean.evaluation.targets.front();
  const eval::TargetScore& b = twice.evaluation.targets.front();
  EXPECT_EQ(b.track_ids.size(), 1u);
  EXPECT_GT(b.tracked_fraction, 0.95);
  EXPECT_LT(b.rmse_horizontal_m, 1.2 * a.rmse_horizontal_m);
  EXPECT_GT(b.nees_mean, 0.5);
  EXPECT_LT(b.nees_mean, 6.0);
}

TEST(Tracker, BaselineSceneKeepsOneTrackPerTarget) {
  const Outcome& o = baseline_outcome();
  ASSERT_EQ(o.evaluation.targets.size(), 5u);
  for (const eval::TargetScore& t : o.evaluation.targets) {
    SCOPED_TRACE(t.name);
    EXPECT_GT(t.tracked_fraction, 0.9);
    EXPECT_EQ(t.id_switches, 0);
    EXPECT_EQ(t.gaps, 0);
    EXPECT_LT(t.rmse_horizontal_m, 20.0);
  }
  EXPECT_EQ(o.evaluation.false_track_count(), 0);
  EXPECT_LT(o.evaluation.mean_gospa(), 45.0);

  // The drones and the airliner identify themselves (Remote ID, ADS-B) or are
  // seen by a camera; the birds are only seen by radar, which cannot tell.
  EXPECT_GT(score(o, "intruder").class_correct, 0.9);
  EXPECT_GT(score(o, "registered").class_correct, 0.9);
  EXPECT_GT(score(o, "airliner").class_correct, 0.9);

  // The online flight feed arrives seconds late; its reports still update the airliner.
  EXPECT_GT(o.tracker->stats().late_updates, 0u);
}

/// Every sensor except the receivers measures angles through a mount that
/// may be turned from its survey, and stamps time with a clock that may be
/// off. Cooperative targets reveal both, but not every component: a camera
/// that only looks along the horizon cannot tell its pitch. A component
/// counts as learned once its uncertainty is below half its prior; only those
/// are checked against the truth (the prior alone passes a 3 sigma test).
TEST(Tracker, LearnsSensorMountAndClockErrors) {
  const Outcome& o = baseline_outcome();
  const TrackerConfig defaults;
  const Eigen::Vector3d prior_sigma(geo::deg2rad(defaults.mount_accuracy_deg),
                                    geo::deg2rad(defaults.mount_accuracy_deg), defaults.clock_accuracy_s);
  int sensors = 0;
  int learned = 0;
  for (const auto& sensor : baseline().sensors) {
    const auto& model = *o.setup->find(sensor->id());
    if (!model.measures_angles()) continue;
    ++sensors;
    const SensorErrors& e = sensor->common().errors;
    const bool arrival_stamped = sensor->common().timestamps == TimestampSource::Arrival;
    const Eigen::Vector3d truth(e.heading_bias_rad, e.pitch_bias_rad,
                                arrival_stamped ? e.latency_mean_s : e.clock_offset_s);
    const Registration::Estimate est = o.tracker->registration().estimate(sensor->id());
    EXPECT_GT(est.samples, 20) << sensor->id();
    for (int k = 0; k < 3; ++k) {
      const double sigma = std::sqrt(est.P(k, k));
      if (sigma > 0.5 * prior_sigma[k]) continue;
      ++learned;
      const double error = std::abs(est.bias[k] - truth[k]);
      EXPECT_LT(error, 3.0 * sigma) << sensor->id() << " component " << k << ": estimate " << est.bias[k]
                                    << " +- " << sigma << ", truth " << truth[k];
      // A heading error the filter claims to resolve (well beyond its
      // uncertainty) is at least halved.
      if (k == 0 && std::abs(truth[k]) > 3.0 * sigma) {
        EXPECT_LT(error, 0.5 * std::abs(truth[k])) << sensor->id();
      }
    }
  }
  EXPECT_EQ(sensors, 4);  // radar, two cameras, direction finder
  EXPECT_GE(learned, 6);  // of 12

  // The radar sees the airliner and the registered drone all the time, so its
  // 1 degree heading error is pinned down well.
  const auto radar = o.tracker->registration().estimate("radar-1");
  EXPECT_LT(std::sqrt(radar.P(0, 0)), geo::deg2rad(0.2));
}

/// The seed-42 checks above, loosened, must hold for other seeds too: every
/// seed redraws all noise, false alarms, dropouts and latencies.
TEST(Tracker, BaselineSceneHoldsUpAcrossSeeds) {
  int breaks = 0;
  double gospa_sum = 0.0;
  const std::vector<std::uint64_t> seeds{1, 2, 3, 4, 5};
  for (const std::uint64_t seed : seeds) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    Scenario scenario = load_scenario(scenario_path("cuas_baseline"));
    scenario.seed = seed;
    const Outcome o = track(scenario, simulate(scenario), TrackerConfig{});
    for (const eval::TargetScore& t : o.evaluation.targets) {
      SCOPED_TRACE(t.name);
      EXPECT_GT(t.tracked_fraction, 0.85);
      EXPECT_LT(t.rmse_horizontal_m, 30.0);
      // Position NEES averages 3 for an honest covariance. The airliner is
      // the exception: the radar is calibrated against the airliner's own
      // ADS-B, so it shares part of that GNSS error, and the tracker does
      // not model that coupling.
      EXPECT_LT(t.nees_mean, t.name == "airliner" ? 15.0 : 8.0);
      breaks += t.id_switches + t.gaps;
    }
    EXPECT_LE(o.evaluation.false_track_count(), 1);
    EXPECT_LT(o.evaluation.mean_gospa(), 75.0);
    gospa_sum += o.evaluation.mean_gospa();
  }
  EXPECT_LT(gospa_sum / static_cast<double>(seeds.size()), 50.0);
  // 25 target-runs. Known: a crow circling at 30 deg/s outruns the
  // constant-velocity model after a missed scan (seed 3); the intruder's
  // track is lost while the mount is still uncalibrated (seed 4).
  EXPECT_LE(breaks, 3);
}

/// Consecutive ADS-B reports of one aircraft share its slowly drifting GNSS
/// error. Hundreds of them must not shrink the track's covariance below
/// that error, as they would if each were independent.
TEST(Tracker, BroadcastsDoNotAverageAwayTheirSharedGnssError) {
  const SimulationResult adsb_only = edited(baseline_sim(), [](std::vector<SensorMessage>& messages) {
    std::erase_if(messages, [](const SensorMessage& m) { return m.sensor_id != "adsb-rx"; });
  });
  const Outcome o = track(baseline(), adsb_only, TrackerConfig{});
  const eval::TargetScore& airliner = score(o, "airliner");
  EXPECT_GT(airliner.tracked_fraction, 0.9);
  EXPECT_GT(airliner.nees_mean, 1.5);
  EXPECT_LT(airliner.nees_mean, 5.0);
  // It reports 10 m horizontal accuracy; the track cannot know better than about that.
  double min_std = std::numeric_limits<double>::infinity();
  for (const TrackReport& r : o.tracker->reports()) {
    if (r.identity == "adsb:3c6589" && r.status == TrackStatus::Confirmed) {
      min_std = std::min(min_std, std::sqrt(r.state.P(0, 0)));
    }
  }
  EXPECT_GT(min_std, 8.0);
}

/// A camera's remaining mount and clock error is the same in every frame.
/// Treated as fresh noise per frame, 15 frames a second average it away and
/// the track claims an accuracy it does not have; carried as a consider
/// parameter, it is not. (Seed 2: the PTZ camera's estimate ends up about
/// 2 sigma off, the case where this matters.)
TEST(Tracker, RegistrationErrorIsNotAveragedAwayAcrossFrames) {
  Scenario scenario = load_scenario(scenario_path("cuas_baseline"));
  scenario.seed = 2;
  const SimulationResult sim = simulate(scenario);
  TrackerConfig as_noise;
  as_noise.consider_registration = false;
  const double nees_consider = score(track(scenario, sim, TrackerConfig{}), "intruder").nees_mean;
  const double nees_noise = score(track(scenario, sim, as_noise), "intruder").nees_mean;
  EXPECT_LT(nees_consider, 8.0);
  EXPECT_GT(nees_noise, 2.0 * nees_consider);
}

/// Mean horizontal error of the tracks on one target within [t0, t1].
double mean_error_between(const Outcome& o, int target_id, double t0, double t1) {
  double sum = 0.0;
  int n = 0;
  for (const eval::Match& m : o.evaluation.matches) {
    if (m.target_id == target_id && m.t_s >= t0 && m.t_s <= t1) {
      sum += m.error_horizontal_m;
      ++n;
    }
  }
  EXPECT_GT(n, 0);
  return n > 0 ? sum / n : std::numeric_limits<double>::infinity();
}

/// The intruder hovers from 95 s to 125 s, inside the radar's Doppler notch.
/// The cameras and the RF direction finder still see it, but only in angle.
/// Inferring "range rate ~ 0" from the radar's silence pins the track's
/// velocity, so it stays on the drone instead of drifting along the bearings.
TEST(Tracker, NotchInferenceHoldsAHoveringDroneInPlace) {
  const eval::TargetScore& with = score(baseline_outcome(), "intruder");
  EXPECT_EQ(with.track_ids.size(), 1u);
  EXPECT_GT(with.tracked_fraction, 0.95);
  EXPECT_GT(baseline_outcome().tracker->stats().notch_inferences, 0u);

  TrackerConfig off;
  off.notch_inference_probability = 0.0;
  const Outcome without = track(baseline(), baseline_sim(), off);
  EXPECT_EQ(without.tracker->stats().notch_inferences, 0u);
  // Other sensors still see it, so the radar's silence must not delete it.
  EXPECT_GT(score(without, "intruder").tracked_fraction, 0.9);

  const double hover_with = mean_error_between(baseline_outcome(), with.id, 95.0, 125.0);
  const double hover_without = mean_error_between(without, with.id, 95.0, 125.0);
  EXPECT_LT(hover_with, 15.0);
  EXPECT_LT(hover_with, 0.5 * hover_without);
}

/// Without registration the radar's 1 degree heading error puts targets
/// tens of metres off at a few kilometres; registration halves the error on
/// targets that do not broadcast their position themselves.
TEST(Tracker, RegistrationImprovesNonCooperativeTargets) {
  TrackerConfig off;
  off.registration = false;
  const Outcome without = track(baseline(), baseline_sim(), off);
  for (const std::string name : {"gull", "crow"}) {
    EXPECT_LT(score(baseline_outcome(), name).rmse_horizontal_m, 0.75 * score(without, name).rmse_horizontal_m)
        << name;
  }
}

/// The messages with neighbours that arrived less than 0.5 s apart delivered
/// the other way round. Only messages that arrive well within a 3 s reorder
/// window take part: the online flight feed, seconds late, is handled as
/// out-of-sequence data, which by nature depends on what came before it.
std::vector<SensorMessage> swap_close_arrivals(std::vector<SensorMessage> messages) {
  // A rotating radar's message arrives just after its 1 s scan ends.
  const auto prompt = [](const SensorMessage& m) { return m.t_arrival_s - m.t_true_s < 1.5; };
  int swaps = 0;
  for (std::size_t i = 0; i + 1 < messages.size(); ++i) {
    if (prompt(messages[i]) && prompt(messages[i + 1]) && messages[i + 1].t_arrival_s - messages[i].t_arrival_s < 0.5) {
      std::swap(messages[i], messages[i + 1]);
      ++swaps;
      ++i;
    }
  }
  EXPECT_GT(swaps, 100);
  return messages;
}

/// Messages may arrive in any order within the reorder window. As long as
/// each one arrives within it, the order they arrive in must not change the
/// output at all.
TEST(Tracker, ArrivalOrderWithinTheReorderWindowDoesNotMatter) {
  TrackerConfig config;
  config.reorder_window_s = 3.0;
  // Online registration corrects each message with what it has learned by
  // the time the message arrives (as a live system must), so the exact
  // output depends on arrival order through it. Hold it fixed here.
  config.registration = false;

  const auto setup = std::make_shared<const SensorSetup>(nominal_sensors_json(baseline()), config);
  const Tracker a = run_tracker(setup, config, baseline_sim().messages);
  const Tracker b = run_tracker(setup, config, swap_close_arrivals(baseline_sim().messages));
  ASSERT_EQ(a.reports().size(), b.reports().size());
  for (std::size_t i = 0; i < a.reports().size(); ++i) {
    const TrackReport& ra = a.reports()[i];
    const TrackReport& rb = b.reports()[i];
    ASSERT_EQ(ra.track_id, rb.track_id) << "report " << i;
    ASSERT_EQ(ra.t_s, rb.t_s) << "report " << i;
    ASSERT_EQ(ra.state.x, rb.state.x) << "report " << i;
  }
  EXPECT_EQ(a.stats().updates, b.stats().updates);
}

/// With registration learning, arrival order changes only which correction
/// each message gets, by a sliver of the estimate's own uncertainty: the
/// tracks and their accuracy stay the same.
TEST(Tracker, ArrivalOrderBarelyMattersWithOnlineRegistration) {
  TrackerConfig config;
  config.reorder_window_s = 3.0;
  const Outcome a = track(baseline(), baseline_sim(), config);
  SimulationResult swapped{swap_close_arrivals(baseline_sim().messages), {}};
  const Outcome b = track(baseline(), swapped, config);
  ASSERT_EQ(a.evaluation.targets.size(), b.evaluation.targets.size());
  for (std::size_t i = 0; i < a.evaluation.targets.size(); ++i) {
    const eval::TargetScore& ta = a.evaluation.targets[i];
    const eval::TargetScore& tb = b.evaluation.targets[i];
    SCOPED_TRACE(ta.name);
    EXPECT_EQ(ta.track_ids, tb.track_ids);
    EXPECT_NEAR(ta.tracked_fraction, tb.tracked_fraction, 0.01);
    EXPECT_NEAR(ta.rmse_horizontal_m, tb.rmse_horizontal_m, 0.5);
  }
  EXPECT_EQ(a.evaluation.false_track_count(), b.evaluation.false_track_count());
}

/// What the data stream itself reveals about each sensor. Lost messages show
/// up as gaps in the sequence counter, and must match what the simulated
/// network dropped (except drops after a sensor's last delivered message).
TEST(Tracker, ReportsSensorHealthFromTheDataStream) {
  const Outcome& o = baseline_outcome();
  for (const SensorStats& sim : baseline_sim().stats) {
    SCOPED_TRACE(sim.sensor_id);
    const SensorHealth& h = o.tracker->health().at(sim.sensor_id);
    EXPECT_EQ(h.messages, sim.messages);
    EXPECT_LE(h.lost_messages, sim.dropped);
    EXPECT_GE(h.lost_messages + 2, sim.dropped);
    EXPECT_EQ(h.out_of_order, 0u);
    EXPECT_NEAR(h.rate_hz(), sim.messages / 180.0, 0.05 * sim.messages / 180.0 + 0.01);
  }
  // The online flight feed is seconds behind; everything else arrives within the window.
  const auto& health = o.tracker->health();
  EXPECT_EQ(health.at("osint-feed").late_messages, health.at("osint-feed").messages);
  for (const char* id : {"radar-1", "cam-wide", "cam-ptz", "adsb-rx"}) EXPECT_EQ(health.at(id).late_messages, 0u) << id;
}

/// One malformed message must not stop a tracker in the field: it is
/// counted and skipped, and the tracks carry on.
TEST(Tracker, SkipsBadInputAndCarriesOn) {
  std::vector<SensorMessage> messages = baseline_sim().messages;
  const auto radar_with_plots = std::find_if(messages.begin() + 100, messages.end(), [](const SensorMessage& m) {
    return m.sensor_id == "radar-1" && !m.detections.empty();
  });
  ASSERT_NE(radar_with_plots, messages.end());
  std::get<RadarDetection>(radar_with_plots->detections.front().payload).range_m =
      std::numeric_limits<double>::quiet_NaN();
  SensorMessage stranger = messages[50];
  stranger.sensor_id = "radar-9";
  messages.insert(messages.begin() + 51, stranger);

  const TrackerConfig config;
  const auto setup = std::make_shared<const SensorSetup>(nominal_sensors_json(baseline()), config);
  Tracker tracker = run_tracker(setup, config, messages);
  EXPECT_EQ(tracker.stats().rejected_messages, 1u);
  EXPECT_EQ(tracker.stats().rejected_measurements, 1u);
  EXPECT_EQ(tracker.stats().tracks_confirmed, baseline_outcome().tracker->stats().tracks_confirmed);
  for (const TrackReport& r : tracker.reports()) ASSERT_TRUE(r.state.x.allFinite());
}

/// A corrupt line in a recording (a truncated write, a sensor bug) is
/// skipped and reported, not fatal: the rest of the recording still counts.
TEST(TrackIo, SkipsCorruptMeasurementLines) {
  const auto path = std::filesystem::temp_directory_path() / "skytrack_corrupt_measurements.jsonl";
  {
    std::ofstream out(path);
    for (std::size_t i = 0; i < 3; ++i) {
      out << to_observed_json(baseline_sim().messages[i]).dump() << "\n";
      if (i == 0) out << "{\"sensor_id\": \"radar-1\", \"seq\":\n";  // truncated
      if (i == 1) out << "not json at all\n";
    }
  }
  const MeasurementLog log = read_measurements(path);
  std::filesystem::remove(path);
  EXPECT_EQ(log.messages.size(), 3u);
  EXPECT_EQ(log.bad_lines, 2u);
  ASSERT_EQ(log.first_errors.size(), 2u);
  EXPECT_NE(log.first_errors[0].find(":2:"), std::string::npos) << log.first_errors[0];
}

/// tracks.csv carries the whole state covariance, cross terms included,
/// so a node downstream (track-to-track fusion) can use it as is.
TEST(TrackIo, TracksFileRoundTripsTheFullCovariance) {
  const auto dir = std::filesystem::temp_directory_path() / "skytrack_tracks_roundtrip";
  std::filesystem::remove_all(dir);
  const Tracker& tracker = *baseline_outcome().tracker;
  write_tracker_output(tracker, dir);
  const std::vector<TrackReport> back = read_track_reports(dir / "tracks.csv");
  std::filesystem::remove_all(dir);
  ASSERT_EQ(back.size(), tracker.reports().size());
  double worst = 0.0;
  for (std::size_t i = 0; i < back.size(); ++i) {
    const Mat6& a = tracker.reports()[i].state.P;
    const Mat6& b = back[i].state.P;
    worst = std::max(worst, ((a - b).cwiseAbs().array() / (a.cwiseAbs().array() + 1e-9)).maxCoeff());
  }
  EXPECT_LT(worst, 1e-8);  // 10 significant digits
  // The position-velocity cross terms are there, not zeros.
  EXPECT_NE(back.back().state.P(0, 3), 0.0);
}

/// Reports wait for the reorder window, so they reach users about that
/// much after the time they describe. This is the price of processing
/// measurements in time order; the README discusses the trade-off.
TEST(Tracker, PublishesReportsOneReorderWindowAfterTheirTime) {
  const Tracker& tracker = *baseline_outcome().tracker;
  const Json latency = report_latency_json(tracker);
  EXPECT_GE(latency["p50"].get<double>(), tracker.config().reorder_window_s);
  EXPECT_LT(latency["p95"].get<double>(), tracker.config().reorder_window_s + 0.5);
  for (const TrackReport& r : tracker.reports()) ASSERT_GE(r.t_published_s, r.t_s) << "never publish the future";
}

/// A short reorder window cuts latency, but then a rotating radar's scans
/// (which arrive about a second after they start) are always behind the
/// cameras. They must still be allowed to start tracks, or new targets wait
/// for a camera or a broadcast to be noticed at all.
TEST(Tracker, ShortReorderWindowStillStartsTracksPromptly) {
  TrackerConfig short_window;
  short_window.reorder_window_s = 0.3;
  TrackerConfig no_late_start = short_window;
  no_late_start.late_initiation_s = 0.0;
  const Outcome prompt = track(baseline(), baseline_sim(), short_window);
  const Outcome slow = track(baseline(), baseline_sim(), no_late_start);

  // From the moment a target appears to its first confirmed report.
  const auto worst_time_to_track = [](const Outcome& o) {
    double worst = 0.0;
    for (const eval::TargetScore& t : o.evaluation.targets) {
      if (!t.first_track_s) return std::numeric_limits<double>::infinity();
      const auto target = std::find_if(baseline().targets.begin(), baseline().targets.end(),
                                       [&](const Target& x) { return x.id == t.id; });
      const double appears = std::max(target->trajectory.t_start(), o.tracker->output_span()->first);
      worst = std::max(worst, *t.first_track_s - appears);
    }
    return worst;
  };
  EXPECT_LT(worst_time_to_track(prompt), 10.0);
  EXPECT_GT(worst_time_to_track(slow), 20.0);
  EXPECT_LT(prompt.evaluation.mean_gospa(), 50.0);
  EXPECT_LT(report_latency_json(*prompt.tracker)["p50"].get<double>(), 0.5);
}

/// A scene with only clutter: radar false alarms, camera false detections
/// and spurious RF bearings, but nothing there. Two-of-three confirmation
/// lets a few chance alignments through for a few seconds; three-of-four
/// (configs/strict.json) lets none through but confirms real targets about a
/// scan later.
TEST(Tracker, ClutterOnlyTracksAreRareAndShortLived) {
  Json j = read_json_file(scenario_path("cuas_baseline"));
  j["targets"] = Json::array();
  TrackerConfig strict;
  strict.confirm_hits = 3;
  strict.confirm_window = 4;

  int confirmed = 0;
  for (const std::uint64_t seed : {1u, 2u, 3u, 4u}) {
    j["seed"] = seed;
    const Scenario scenario = parse_scenario(j);
    const SimulationResult sim = simulate(scenario);
    const Outcome o = track(scenario, sim, TrackerConfig{});
    std::map<int, std::pair<double, double>> lifetime;  // confirmed track -> first and last report
    for (const TrackReport& r : o.tracker->reports()) {
      if (r.status != TrackStatus::Confirmed) continue;
      auto [it, inserted] = lifetime.try_emplace(r.track_id, r.t_s, r.t_s);
      it->second.second = r.t_s;
    }
    for (const auto& [id, span] : lifetime) {
      EXPECT_LT(span.second - span.first, 15.0) << "seed " << seed << " track " << id;
    }
    confirmed += static_cast<int>(lifetime.size());
    EXPECT_EQ(track(scenario, sim, strict).tracker->stats().tracks_confirmed, 0) << "seed " << seed;
  }
  EXPECT_LE(confirmed, 3) << "in 12 minutes of clutter";
}

}  // namespace
}  // namespace skytrack
