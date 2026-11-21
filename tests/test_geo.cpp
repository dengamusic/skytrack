#include <gtest/gtest.h>

#include <cmath>

#include "skytrack/geo.hpp"
#include "skytrack/rng.hpp"

using namespace skytrack;
using namespace skytrack::geo;

TEST(Geo, WrapAngles) {
  EXPECT_NEAR(wrap_pi(1.5 * kPi), -0.5 * kPi, 1e-12);
  EXPECT_NEAR(wrap_pi(-1.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_DOUBLE_EQ(wrap_pi(kPi), kPi);
  EXPECT_DOUBLE_EQ(wrap_pi(-kPi), kPi);  // the range is (-pi, pi]
  EXPECT_NEAR(wrap_2pi(-0.5), 2.0 * kPi - 0.5, 1e-12);
  EXPECT_LT(wrap_2pi(-1e-18), 2.0 * kPi);
}

TEST(Geo, EcefKnownPoints) {
  const Vec3 equator = geodetic_to_ecef({0.0, 0.0, 0.0});
  EXPECT_NEAR(equator.x(), kWgs84A, 1e-9);
  EXPECT_NEAR(equator.y(), 0.0, 1e-9);
  EXPECT_NEAR(equator.z(), 0.0, 1e-9);

  const Vec3 pole = geodetic_to_ecef({kPi / 2.0, 0.0, 0.0});
  EXPECT_NEAR(pole.z(), kWgs84A * (1.0 - kWgs84F), 1e-6);  // semi-minor axis
  EXPECT_NEAR(pole.x(), 0.0, 1e-6);
}

TEST(Geo, EcefRoundTrip) {
  Rng rng(7);
  for (int i = 0; i < 2000; ++i) {
    const Geodetic g{deg2rad(rng.uniform(-89.0, 89.0)), deg2rad(rng.uniform(-180.0, 180.0)),
                     rng.uniform(-500.0, 30000.0)};
    const Geodetic back = ecef_to_geodetic(geodetic_to_ecef(g));
    EXPECT_NEAR(back.lat_rad, g.lat_rad, 1e-11);
    EXPECT_NEAR(wrap_pi(back.lon_rad - g.lon_rad), 0.0, 1e-12);
    EXPECT_NEAR(back.alt_m, g.alt_m, 1e-6);
  }
}

TEST(Geo, LocalFrameAxesPointEastNorthUp) {
  const Geodetic o{deg2rad(48.25), deg2rad(11.65), 480.0};
  const LocalFrame frame(o);
  EXPECT_LT(frame.geodetic_to_enu(o).norm(), 1e-6);

  const Vec3 up = frame.geodetic_to_enu({o.lat_rad, o.lon_rad, o.alt_m + 100.0});
  EXPECT_NEAR(up.z(), 100.0, 1e-6);
  EXPECT_NEAR(up.head<2>().norm(), 0.0, 1e-6);

  const Vec3 north = frame.geodetic_to_enu({o.lat_rad + 1e-5, o.lon_rad, o.alt_m});
  EXPECT_GT(north.y(), 60.0);  // 1e-5 rad of latitude is about 64 m
  EXPECT_NEAR(north.x(), 0.0, 1e-6);

  const Vec3 east = frame.geodetic_to_enu({o.lat_rad, o.lon_rad + 1e-5, o.alt_m});
  EXPECT_GT(east.x(), 40.0);
  EXPECT_NEAR(east.y(), 0.0, 1e-3);  // a parallel curves away from the tangent plane slightly
}

TEST(Geo, LocalFrameRoundTrip) {
  const LocalFrame frame(Geodetic{deg2rad(48.25), deg2rad(11.65), 480.0});
  Rng rng(11);
  for (int i = 0; i < 500; ++i) {
    const Vec3 enu(rng.uniform(-50e3, 50e3), rng.uniform(-50e3, 50e3), rng.uniform(-100.0, 12e3));
    EXPECT_LT((frame.geodetic_to_enu(frame.enu_to_geodetic(enu)) - enu).norm(), 1e-6);
  }
}

TEST(Geo, BodyRotationFollowsCompassHeading) {
  // Facing east: boresight = east, left = north.
  const Mat3 east = rotation_world_from_body(deg2rad(90.0), 0.0, 0.0);
  EXPECT_TRUE((east * Vec3::UnitX()).isApprox(Vec3::UnitX(), 1e-12));
  EXPECT_TRUE((east * Vec3::UnitY()).isApprox(Vec3::UnitY(), 1e-12));

  // Facing north, pitched up 30 degrees.
  const Mat3 up = rotation_world_from_body(0.0, deg2rad(30.0), 0.0);
  EXPECT_TRUE((up * Vec3::UnitX()).isApprox(Vec3(0.0, std::cos(deg2rad(30.0)), std::sin(deg2rad(30.0))), 1e-12));
  EXPECT_NEAR(up.determinant(), 1.0, 1e-12);
  EXPECT_TRUE((up.transpose() * up).isIdentity(1e-12));
}

TEST(Geo, SphericalRoundTripAndBearing) {
  const Vec3 v(120.0, -40.0, 30.0);
  EXPECT_TRUE(spherical_to_cartesian(cartesian_to_spherical(v)).isApprox(v, 1e-12));
  EXPECT_NEAR(compass_bearing({1.0, 0.0, 0.0}), kPi / 2.0, 1e-12);
  EXPECT_NEAR(compass_bearing({0.0, -1.0, 0.0}), kPi, 1e-12);
  EXPECT_NEAR(compass_bearing({-1.0, 0.0, 0.0}), 1.5 * kPi, 1e-12);
}
