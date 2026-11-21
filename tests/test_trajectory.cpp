#include <gtest/gtest.h>

#include <stdexcept>

#include "skytrack/trajectory.hpp"

using namespace skytrack;
using geo::deg2rad;
using geo::kPi;

TEST(Trajectory, ConstantVelocity) {
  const Trajectory tr(0.0, 100.0, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {});
  const KinematicState s = tr.state(10.0);
  EXPECT_TRUE(s.position.isApprox(Vec3(41.0, 52.0, 63.0), 1e-12));
  EXPECT_TRUE(s.velocity.isApprox(Vec3(4.0, 5.0, 6.0), 1e-12));
}

TEST(Trajectory, CoordinatedTurnFliesACircle) {
  const double w = deg2rad(10.0);
  const double period = 2.0 * kPi / w;
  const double speed = 20.0;
  const Trajectory tr(0.0, 1000.0, {0.0, 0.0, 100.0}, {speed, 0.0, 0.0},
                      {{MotionModel::CoordinatedTurn, period, Vec3::Zero(), w}});

  // Back where it started after one full turn.
  EXPECT_LT((tr.state(period).position - Vec3(0.0, 0.0, 100.0)).norm(), 1e-9);
  EXPECT_LT((tr.state(period).velocity - Vec3(speed, 0.0, 0.0)).norm(), 1e-9);

  // Constant speed and centripetal acceleration all the way round.
  for (double t = 0.0; t < period; t += 1.7) {
    const KinematicState s = tr.state(t);
    EXPECT_NEAR(s.velocity.norm(), speed, 1e-9);
    EXPECT_NEAR(s.acceleration.norm(), speed * w, 1e-9);
  }

  // Positive rate turns left: heading east, a quarter turn later it is at
  // (r, r) heading north, with r = v / w.
  const double r = speed / w;
  EXPECT_LT((tr.state(period / 4.0).position - Vec3(r, r, 100.0)).norm(), 1e-9);
}

TEST(Trajectory, ContinuousAcrossSegments) {
  const Trajectory tr(5.0, 200.0, {0.0, 0.0, 50.0}, {10.0, 0.0, 0.0},
                      {{MotionModel::ConstantVelocity, 20.0, Vec3::Zero(), 0.0},
                       {MotionModel::ConstantAcceleration, 5.0, Vec3(-2.0, 1.0, 0.5), 0.0},
                       {MotionModel::CoordinatedTurn, 15.0, Vec3::Zero(), deg2rad(-12.0)}});
  for (const double boundary : {25.0, 30.0, 45.0}) {
    const KinematicState before = tr.state(boundary - 1e-7);
    const KinematicState after = tr.state(boundary + 1e-7);
    EXPECT_LT((before.position - after.position).norm(), 1e-4) << "at t = " << boundary;
    EXPECT_LT((before.velocity - after.velocity).norm(), 1e-4) << "at t = " << boundary;
  }
}

TEST(Trajectory, ActiveWindowAndClamping) {
  const Trajectory tr(5.0, 200.0, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {});
  EXPECT_FALSE(tr.active(4.9));
  EXPECT_TRUE(tr.active(5.0));
  EXPECT_TRUE(tr.active(200.0));
  EXPECT_FALSE(tr.active(200.1));
  EXPECT_TRUE(tr.state(1000.0).position.isApprox(tr.state(200.0).position));
}

TEST(Trajectory, RejectsBadInput) {
  EXPECT_THROW(Trajectory(10.0, 10.0, Vec3::Zero(), Vec3::Zero(), {}), std::invalid_argument);
  EXPECT_THROW(Trajectory(0.0, 10.0, Vec3::Zero(), Vec3::Zero(),
                          {{MotionModel::ConstantVelocity, 0.0, Vec3::Zero(), 0.0}}),
               std::invalid_argument);
}
