#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "skytrack/sensors/camera.hpp"
#include "skytrack/sensors/cooperative.hpp"
#include "skytrack/sensors/radar.hpp"
#include "skytrack/sensors/rf_bearing.hpp"
#include "test_helpers.hpp"

using namespace skytrack;
using geo::deg2rad;
using geo::kPi;

namespace {

const geo::LocalFrame& frame() {
  static const geo::LocalFrame f(geo::Geodetic{deg2rad(48.25), deg2rad(11.65), 480.0});
  return f;
}

/// Sees everything, everywhere, exactly, with no false alarms.
RadarParams ideal_radar() {
  RadarParams p;
  p.mode = RadarParams::Mode::Staring;
  p.min_range_m = 1.0;
  p.max_range_m = 1e5;
  p.fov_az_rad = 2.0 * kPi;
  p.min_el_rad = -kPi / 2.0;
  p.max_el_rad = kPi / 2.0;
  p.sigma_range_m = 0.0;
  p.sigma_az_rad = 0.0;
  p.sigma_el_rad = 0.0;
  p.sigma_range_rate_mps = 0.0;
  p.snr_ref_db = 200.0;
  p.false_alarms_per_scan = 0.0;
  p.doppler_notch_mps = 0.0;
  return p;
}

CameraParams ideal_camera() {
  CameraParams p;
  p.width_px = 1920;
  p.height_px = 1080;
  p.fx_px = 2000.0;
  p.fy_px = 2000.0;
  p.cx_px = 960.0;
  p.cy_px = 540.0;
  p.sigma_px = 0.0;
  p.size_noise_frac = 0.0;
  p.pd_max = 1.0;
  p.size_at_half_pd_px = -1000.0;  // detection probability ~1 for anything above min_size_px
  p.max_range_m = 1e5;
  p.false_alarms_per_frame = 0.0;
  p.class_accuracy = 1.0;
  return p;
}

const RadarDetection& radar_payload(const Detection& d) { return std::get<RadarDetection>(d.payload); }

}  // namespace

// ---- radar ------------------------------------------------------------------

TEST(Radar, MeasuresGeometryInBodyFrame) {
  const Radar radar(test::common("r"), ideal_radar());
  const Vec3 p(100.0, 200.0, 50.0);
  const Vec3 v(3.0, 4.0, 0.0);
  const std::vector<Target> targets{test::straight_target(1, p, v)};
  Rng rng(1);
  const Scan scan = radar.observe(0.0, World{targets, frame()}, rng);
  ASSERT_EQ(scan.detections.size(), 1u);
  const RadarDetection& d = radar_payload(scan.detections[0]);
  EXPECT_NEAR(d.range_m, p.norm(), 1e-9);
  // Facing north: east is to the right, so it has a negative body azimuth.
  EXPECT_NEAR(d.az_rad, std::atan2(-100.0, 200.0), 1e-12);
  EXPECT_NEAR(d.el_rad, std::atan2(50.0, std::hypot(100.0, 200.0)), 1e-12);
  EXPECT_NEAR(d.range_rate_mps, p.dot(v) / p.norm(), 1e-12);
  EXPECT_EQ(scan.detections[0].origin_target_id, 1);
}

TEST(Radar, HeadingBiasShiftsAzimuth) {
  // The radar is really turned 1 degree clockwise, so a target due north
  // appears 1 degree to the left of where the nominal mount says it should.
  SensorCommon c = test::common("r");
  c.errors.heading_bias_rad = deg2rad(1.0);
  const Radar radar(c, ideal_radar());
  const std::vector<Target> targets{test::straight_target(1, {0.0, 1000.0, 0.0})};
  Rng rng(1);
  const Scan scan = radar.observe(0.0, World{targets, frame()}, rng);
  ASSERT_EQ(scan.detections.size(), 1u);
  EXPECT_NEAR(radar_payload(scan.detections[0]).az_rad, deg2rad(1.0), 1e-12);
}

TEST(Radar, NoiseIsStatedSigmaTimesScale) {
  SensorCommon c = test::common("r");
  c.errors.noise_scale = 2.0;
  RadarParams p = ideal_radar();
  p.sigma_range_m = 5.0;
  p.sigma_az_rad = deg2rad(0.5);
  const Radar radar(c, p);
  const Vec3 position(0.0, 2000.0, 100.0);
  const std::vector<Target> targets{test::straight_target(1, position)};
  Rng rng(3);
  const int n = 20000;
  double sum = 0.0;
  double sum_sq = 0.0;
  double az_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const Scan scan = radar.observe(0.0, World{targets, frame()}, rng);
    const RadarDetection& d = radar_payload(scan.detections.at(0));
    const double e = d.range_m - position.norm();
    sum += e;
    sum_sq += e * e;
    az_sq += d.az_rad * d.az_rad;
  }
  EXPECT_NEAR(sum / n, 0.0, 0.3);
  EXPECT_NEAR(std::sqrt(sum_sq / n), 10.0, 0.3);
  EXPECT_NEAR(std::sqrt(az_sq / n), deg2rad(1.0), deg2rad(0.03));
}

TEST(Radar, DetectionProbabilityFollowsSwerlingOne) {
  RadarParams p = ideal_radar();
  p.snr_ref_db = 10.0;
  p.ref_range_m = 1000.0;
  p.pfa = 1e-6;
  const Radar radar(test::common("r"), p);
  const double expected = std::pow(1e-6, 1.0 / 11.0);  // SNR 10 dB = 10x
  EXPECT_NEAR(radar.detection_probability(10.0), expected, 1e-12);

  const std::vector<Target> targets{test::straight_target(1, {0.0, 1000.0, 0.0})};  // 0 dBsm at ref range
  Rng rng(5);
  const int n = 20000;
  std::size_t hits = 0;
  for (int i = 0; i < n; ++i) hits += radar.observe(0.0, World{targets, frame()}, rng).detections.size();
  EXPECT_NEAR(static_cast<double>(hits) / n, expected, 0.015);
}

TEST(Radar, FalseAlarmsArePoissonAndInsideCoverage) {
  RadarParams p = ideal_radar();
  p.false_alarms_per_scan = 3.0;
  p.min_range_m = 100.0;
  p.max_range_m = 5000.0;
  p.fov_az_rad = deg2rad(90.0);
  p.min_el_rad = 0.0;
  p.max_el_rad = deg2rad(20.0);
  p.doppler_notch_mps = 1.0;
  p.max_range_rate_mps = 30.0;
  const Radar radar(test::common("r"), p);
  const std::vector<Target> none;
  Rng rng(9);
  const int n = 20000;
  std::size_t total = 0;
  std::size_t outside = 0;
  for (int i = 0; i < n; ++i) {
    for (const Detection& det : radar.observe(0.0, World{none, frame()}, rng).detections) {
      const RadarDetection& d = radar_payload(det);
      ++total;
      const bool inside = det.origin_target_id == -1 && d.range_m >= 100.0 && d.range_m <= 5000.0 &&
                          std::abs(d.az_rad) <= deg2rad(45.0) && d.el_rad >= 0.0 && d.el_rad <= deg2rad(20.0) &&
                          std::abs(d.range_rate_mps) >= 1.0 && std::abs(d.range_rate_mps) <= 30.0;
      if (!inside) ++outside;
    }
  }
  EXPECT_EQ(outside, 0u);
  EXPECT_NEAR(static_cast<double>(total) / n, 3.0, 0.05);
}

TEST(Radar, IgnoresTargetsOutsideFieldOfView) {
  RadarParams p = ideal_radar();
  p.fov_az_rad = deg2rad(90.0);
  const Radar radar(test::common("r"), p);
  const std::vector<Target> targets{test::straight_target(1, {0.0, -1000.0, 0.0})};  // behind
  Rng rng(1);
  EXPECT_TRUE(radar.observe(0.0, World{targets, frame()}, rng).detections.empty());
}

TEST(Radar, DopplerNotchHidesHoveringTargets) {
  RadarParams p = ideal_radar();
  p.doppler_notch_mps = 1.0;
  const Radar radar(test::common("r"), p);
  Rng rng(1);
  const std::vector<Target> hovering{test::straight_target(1, {0.0, 1000.0, 50.0})};
  EXPECT_TRUE(radar.observe(0.0, World{hovering, frame()}, rng).detections.empty());
  const std::vector<Target> approaching{test::straight_target(1, {0.0, 1000.0, 50.0}, {0.0, -5.0, 0.0})};
  EXPECT_EQ(radar.observe(0.0, World{approaching, frame()}, rng).detections.size(), 1u);
}

TEST(Radar, RotatingBeamTimesFollowAzimuth) {
  RadarParams p = ideal_radar();
  p.mode = RadarParams::Mode::Rotating;
  const Radar radar(test::common("r", 1.0), p);
  // Static targets at compass bearings 10, 100, 190 and 280 degrees.
  std::vector<Target> targets;
  const std::vector<double> bearings{10.0, 100.0, 190.0, 280.0};
  for (std::size_t i = 0; i < bearings.size(); ++i) {
    targets.push_back(test::straight_target(static_cast<int>(i), 1000.0 * test::direction(deg2rad(bearings[i]), 0.0)));
  }
  Rng rng(1);
  const Scan scan = radar.observe(10.0, World{targets, frame()}, rng);
  EXPECT_DOUBLE_EQ(scan.t_ready_s, 11.0);  // sent when the sweep is complete
  ASSERT_EQ(scan.detections.size(), 4u);
  for (const Detection& d : scan.detections) {
    const double bearing = bearings[static_cast<std::size_t>(d.origin_target_id)];
    EXPECT_NEAR(d.t_true_s, 10.0 + bearing / 360.0, 1e-9);  // clockwise sweep from north
  }
  EXPECT_TRUE(std::is_sorted(scan.detections.begin(), scan.detections.end(),
                             [](const Detection& a, const Detection& b) { return a.t_true_s < b.t_true_s; }));
}

// ---- camera -----------------------------------------------------------------

TEST(Camera, BoresightHitsPrincipalPoint) {
  const Camera cam(test::common("c"), ideal_camera());
  const auto px = cam.project({0.0, 500.0, 0.0}, 0.0);
  ASSERT_TRUE(px);
  EXPECT_NEAR(px->x(), 960.0, 1e-6);
  EXPECT_NEAR(px->y(), 540.0, 1e-6);
}

TEST(Camera, AnglesMapThroughFocalLength) {
  const Camera cam(test::common("c"), ideal_camera());
  const double b = deg2rad(10.0);  // right of boresight
  const double e = deg2rad(5.0);   // above boresight
  const auto px = cam.project(1000.0 * test::direction(b, e), 0.0);
  ASSERT_TRUE(px);
  EXPECT_NEAR(px->x(), 960.0 + 2000.0 * std::tan(b), 1e-6);
  EXPECT_NEAR(px->y(), 540.0 - 2000.0 * std::tan(e) / std::cos(b), 1e-6);
}

TEST(Camera, GimbalPanAndTiltCentreTheTarget) {
  CameraParams p = ideal_camera();
  p.gimbal = {{0.0, deg2rad(10.0), deg2rad(5.0)}};
  const Camera cam(test::common("c"), p);
  const std::vector<Target> targets{
      test::straight_target(1, 1000.0 * test::direction(deg2rad(10.0), deg2rad(5.0)), Vec3::Zero(),
                            TargetClass::Aircraft)};
  Rng rng(1);
  const Scan scan = cam.observe(0.0, World{targets, frame()}, rng);
  ASSERT_TRUE(scan.gimbal);
  EXPECT_NEAR(scan.gimbal->pan_rad, deg2rad(10.0), 1e-12);
  ASSERT_EQ(scan.detections.size(), 1u);
  const auto& d = std::get<CameraDetection>(scan.detections[0].payload);
  EXPECT_NEAR(d.u_px, 960.0, 1e-6);
  EXPECT_NEAR(d.v_px, 540.0, 1e-6);
  EXPECT_EQ(d.label, "aircraft");
}

TEST(Camera, MissesTargetsBehindOutsideOrTooSmall) {
  const Camera cam(test::common("c"), ideal_camera());
  EXPECT_FALSE(cam.project({0.0, -500.0, 0.0}, 0.0));                                // behind
  EXPECT_FALSE(cam.project(1000.0 * test::direction(deg2rad(60.0), 0.0), 0.0));      // left of the frame edge
  const std::vector<Target> tiny{test::straight_target(1, {0.0, 1000.0, 0.0}, Vec3::Zero(), TargetClass::Drone,
                                                       0.0, 0.001)};  // 0.002 px
  Rng rng(1);
  EXPECT_TRUE(cam.observe(0.0, World{tiny, frame()}, rng).detections.empty());
}

// ---- RF direction finder ----------------------------------------------------

TEST(RfBearing, HearsOnlyEmitters) {
  RfBearingParams p;
  p.sigma_az_rad = 0.0;
  p.pd = 1.0;
  p.false_alarms_per_report = 0.0;
  p.signature_id_prob = 1.0;
  const RfBearingSensor df(test::common("df"), p);
  Target emitter = test::straight_target(1, {500.0, 500.0, 50.0});
  emitter.rf_signature = "ocusync";
  const std::vector<Target> targets{emitter, test::straight_target(2, {-500.0, 500.0, 50.0})};
  Rng rng(1);
  const Scan scan = df.observe(0.0, World{targets, frame()}, rng);
  ASSERT_EQ(scan.detections.size(), 1u);
  const auto& d = std::get<BearingDetection>(scan.detections[0].payload);
  EXPECT_NEAR(d.az_rad, -kPi / 4.0, 1e-12);  // north-east is 45 degrees right of a north-facing antenna
  EXPECT_EQ(d.signature, "ocusync");
}

// ---- cooperative receivers --------------------------------------------------

TEST(Cooperative, ReportsBroadcastPositionInWgs84) {
  CooperativeParams p;
  p.protocol = "adsb";
  p.reception_prob = 1.0;
  p.velocity_std_mps = 0.0;
  const CooperativeReceiver rx(test::common("adsb", 2.0), p);

  Target plane = test::straight_target(7, {1000.0, 2000.0, 900.0}, {50.0, 0.0, 0.0}, TargetClass::Aircraft);
  plane.cooperative = CooperativeBroadcast{"adsb", "3c6589", 10.0, 15.0, {0.1, 0.7, 1.3, 2.0, 2.9, 3.3, 4.1, 5.0, 5.9}};
  Target drone = test::straight_target(8, {300.0, 300.0, 50.0});
  drone.cooperative = CooperativeBroadcast{"remote_id", "1596F350457", 3.0, 5.0, {}};
  const std::vector<Target> targets{plane, drone};

  Rng rng(4);
  const Scan scan = rx.observe(100.0, World{targets, frame()}, rng);
  ASSERT_EQ(scan.detections.size(), 1u);  // the Remote ID drone is not heard by an ADS-B receiver
  const Detection& det = scan.detections[0];
  EXPECT_EQ(det.origin_target_id, 7);
  EXPECT_LE(det.t_true_s, 100.0);
  EXPECT_GT(det.t_true_s, 99.5);  // broadcast within the last period

  const auto& r = std::get<CooperativeReport>(det.payload);
  EXPECT_EQ(r.identity, "3c6589");
  const Vec3 reported = frame().geodetic_to_enu({deg2rad(r.lat_deg), deg2rad(r.lon_deg), r.alt_m});
  const Vec3 expected = plane.trajectory.state(det.t_true_s).position + plane.navigation_error(det.t_true_s);
  EXPECT_LT((reported - expected).norm(), 1e-6);
  EXPECT_NEAR(r.vel_east_mps, 50.0, 1e-12);
}

TEST(Cooperative, NavigationErrorHasStatedSpread) {
  Target plane = test::straight_target(7, Vec3::Zero());
  plane.cooperative = CooperativeBroadcast{"adsb", "x", 10.0, 15.0, {0.1, 0.7, 1.3, 2.0, 2.9, 3.3, 4.1, 5.0, 5.9}};
  double sum_x = 0.0;
  double sum_z = 0.0;
  int n = 0;
  for (double t = 0.0; t < 50000.0; t += 1.0, ++n) {
    const Vec3 e = plane.navigation_error(t);
    sum_x += e.x() * e.x();
    sum_z += e.z() * e.z();
  }
  EXPECT_NEAR(std::sqrt(sum_x / n), 10.0, 0.5);
  EXPECT_NEAR(std::sqrt(sum_z / n), 15.0, 0.75);
}

// ---- common -----------------------------------------------------------------

TEST(Sensor, ClockAddsOffsetAndDrift) {
  SensorCommon c = test::common("r");
  c.errors.clock_offset_s = 0.08;
  c.errors.clock_drift_ppm = 10.0;
  const Radar radar(c, ideal_radar());
  EXPECT_NEAR(radar.sensor_clock(100.0), 100.0 + 0.08 + 0.001, 1e-12);
}
