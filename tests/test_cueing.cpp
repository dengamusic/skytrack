#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <optional>
#include <string_view>
#include <vector>

#include "skytrack/cueing/cueing.hpp"
#include "skytrack/eval/metrics.hpp"
#include "skytrack/geo.hpp"
#include "skytrack/run_writer.hpp"
#include "skytrack/sensors/camera.hpp"
#include "skytrack/tracking/track_io.hpp"
#include "test_helpers.hpp"

namespace skytrack {
namespace {

using geo::deg2rad;

Mount tilted_mount() {
  Mount m;
  m.position_enu = Vec3(10.0, -20.0, 5.0);
  m.heading_rad = deg2rad(45.0);
  m.pitch_rad = deg2rad(5.0);
  return m;
}

TrackReport report_at(int id, const Vec3& p, const std::string& cls = "unknown") {
  TrackReport r;
  r.track_id = id;
  r.status = TrackStatus::Confirmed;
  r.classification = cls;
  r.state.x.head<3>() = p;
  r.state.x.tail<3>().setZero();
  r.state.P = Mat6::Identity();
  return r;
}

/// The controller's pointing and the camera model agree: pointing at a
/// point puts it in the centre of the image.
TEST(CueController, PointingPutsTheTargetInTheImageCentre) {
  SensorCommon c = test::common("ptz", 10.0);
  c.mount = tilted_mount();
  CameraParams params;
  params.fx_px = params.fy_px = 20000.0;
  params.gimbal = {{0.0, 0.0, 0.0}};
  const Camera camera(c, params);
  const CueController controller(CueConfig{}, c.mount, {});
  for (const double bearing : {-170.0, -60.0, 0.0, 45.0, 120.0}) {
    for (const double elevation : {-2.0, 3.0, 25.0}) {
      const Vec3 p = c.mount.position_enu + 1500.0 * test::direction(deg2rad(bearing), deg2rad(elevation));
      const auto pixel = camera.project(p, controller.pointing_to(p));
      ASSERT_TRUE(pixel) << bearing << " " << elevation;
      EXPECT_NEAR(pixel->x(), params.cx_px, 1e-6);
      EXPECT_NEAR(pixel->y(), params.cy_px, 1e-6);
    }
  }
}

TEST(CueController, SlewsNoFasterThanTheGimbalCan) {
  CueConfig config;
  config.max_slew_deg_s = 60.0;
  Mount mount;  // looks north
  CueController controller(config, mount, {});
  const std::vector<TrackReport> picture{report_at(1, Vec3(1000.0, 0.0, 0.0))};  // due east: pan 90 deg
  controller.step(0.0, picture);
  const GimbalReport half_second = controller.step(0.5, picture);
  EXPECT_NEAR(half_second.pan_rad, deg2rad(30.0), 1e-9);
  GimbalReport g;
  for (int k = 2; k <= 6; ++k) g = controller.step(0.5 * k, picture);
  EXPECT_NEAR(g.pan_rad, deg2rad(90.0), 1e-9);  // there after 1.5 s
  EXPECT_EQ(controller.target(), 1);
  // Still no class after max_dwell_s on it: the camera lets it go for now.
  for (int k = 7; k <= 20; ++k) controller.step(0.5 * k, picture);
  EXPECT_EQ(controller.target(), std::nullopt);
}

/// Unclassified tracks come first, then custody of a drone; a track that
/// identifies itself (transponder, Remote ID) is never looked at.
TEST(CueController, LooksAtWhatNeedsClassifyingFirst) {
  CueConfig config;
  CueController controller(config, Mount{}, {});
  TrackReport cooperative = report_at(1, Vec3(0.0, 300.0, 50.0));
  cooperative.identity = "adsb:abc123";
  const TrackReport drone = report_at(2, Vec3(0.0, 400.0, 50.0), "drone");
  const TrackReport unknown = report_at(3, Vec3(300.0, 2000.0, 50.0));
  controller.step(0.0, {cooperative, drone, unknown});
  EXPECT_EQ(controller.target(), 3);
  // Once it is classified (a bird), the camera moves on to the drone.
  TrackReport bird = unknown;
  bird.classification = "bird";
  for (int k = 1; k < 100; ++k) controller.step(0.1 * k, {cooperative, drone, bird});
  EXPECT_EQ(controller.target(), 2);
}

/// The baseline scene with the PTZ camera pointed by the tracker. Without
/// knowing where the targets are, it does about as well as the hand-written
/// schedule, and it is the only one that gets the birds classified.
TEST(Cueing, ClosedLoopClassifiesWhatTheScheduleMisses) {
  const std::filesystem::path root = SKYTRACK_SOURCE_DIR;
  const Scenario scenario = load_scenario(root / "scenarios" / "cuas_baseline.json");
  const TrackerConfig config;
  const CuedRun cued = run_cued(scenario, config, CueConfig{});
  const Tracker scripted = run_tracker(std::make_shared<const SensorSetup>(nominal_sensors_json(scenario), config),
                                       config, simulate(scenario).messages);
  const auto score = [&](const Tracker& tracker) {
    const auto span = tracker.output_span();
    return eval::evaluate(eval::truth_from_scenario(scenario), tracker.reports(), span->first,
                          std::min(span->second, scenario.duration_s), config.output_rate_hz, eval::EvalConfig{});
  };
  const eval::Evaluation a = score(scripted);
  const eval::Evaluation b = score(cued.tracker);
  const auto target = [](const eval::Evaluation& e, std::string_view name) -> const eval::TargetScore& {
    for (const auto& t : e.targets) {
      if (t.name == name) return t;
    }
    throw std::runtime_error("no such target");
  };
  for (const char* bird : {"gull", "crow"}) {
    EXPECT_LT(target(a, bird).class_correct, 0.1) << bird;
    EXPECT_GT(target(b, bird).class_correct, 0.8) << bird;
    ASSERT_TRUE(target(b, bird).first_classified_s) << bird;
  }
  EXPECT_LT(b.mean_gospa(), a.mean_gospa() + 5.0);
  EXPECT_GT(static_cast<double>(cued.cue.frames_on_target), 0.7 * static_cast<double>(cued.cue.frames));
}

}  // namespace
}  // namespace skytrack
