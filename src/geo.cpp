#include "skytrack/geo.hpp"

#include <cmath>

namespace skytrack::geo {

double wrap_pi(double angle_rad) {
  double a = std::fmod(angle_rad + kPi, 2.0 * kPi);  // in (-2pi, 2pi)
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

double wrap_2pi(double angle_rad) {
  double a = std::fmod(angle_rad, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  if (a >= 2.0 * kPi) a -= 2.0 * kPi;  // -tiny + 2pi can round up to 2pi
  return a;
}

Vec3 geodetic_to_ecef(const Geodetic& g) {
  const double sin_lat = std::sin(g.lat_rad);
  const double cos_lat = std::cos(g.lat_rad);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  return {(n + g.alt_m) * cos_lat * std::cos(g.lon_rad),
          (n + g.alt_m) * cos_lat * std::sin(g.lon_rad),
          (n * (1.0 - kWgs84E2) + g.alt_m) * sin_lat};
}

Geodetic ecef_to_geodetic(const Vec3& ecef) {
  const double x = ecef.x();
  const double y = ecef.y();
  const double z = ecef.z();
  const double p = std::hypot(x, y);
  // Fixed-point iteration on latitude. Each step shrinks the error by about
  // e^2 (~0.007), so a handful of steps reach machine precision.
  double lat = std::atan2(z, p * (1.0 - kWgs84E2));
  for (int i = 0; i < 8; ++i) {
    const double s = std::sin(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * s * s);
    lat = std::atan2(z + kWgs84E2 * n * s, p);
  }
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  // This form of the height stays accurate near the poles, unlike p/cos(lat) - N.
  const double h = p * cos_lat + z * sin_lat - n * (1.0 - kWgs84E2 * sin_lat * sin_lat);
  return {lat, std::atan2(y, x), h};
}

LocalFrame::LocalFrame(const Geodetic& origin)
    : origin_(origin), origin_ecef_(geodetic_to_ecef(origin)) {
  const double sl = std::sin(origin.lat_rad);
  const double cl = std::cos(origin.lat_rad);
  const double so = std::sin(origin.lon_rad);
  const double co = std::cos(origin.lon_rad);
  R_enu_from_ecef_ << -so, co, 0.0,
                      -sl * co, -sl * so, cl,
                      cl * co, cl * so, sl;
}

Vec3 LocalFrame::ecef_to_enu(const Vec3& ecef) const {
  return R_enu_from_ecef_ * (ecef - origin_ecef_);
}

Vec3 LocalFrame::enu_to_ecef(const Vec3& enu) const {
  return origin_ecef_ + R_enu_from_ecef_.transpose() * enu;
}

Vec3 LocalFrame::geodetic_to_enu(const Geodetic& g) const {
  return ecef_to_enu(geodetic_to_ecef(g));
}

Geodetic LocalFrame::enu_to_geodetic(const Vec3& enu) const {
  return ecef_to_geodetic(enu_to_ecef(enu));
}

Mat3 rotation_world_from_body(double heading_rad, double pitch_rad, double roll_rad) {
  // Body axes start aligned with ENU (boresight east, left north, up up).
  // Yaw turns the boresight counter-clockwise from east, so a compass heading
  // h becomes a yaw of 90deg - h. Pitching up is a negative turn about the
  // left (y) axis.
  const double yaw = kPi / 2.0 - heading_rad;
  return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
          Eigen::AngleAxisd(-pitch_rad, Vec3::UnitY()) *
          Eigen::AngleAxisd(roll_rad, Vec3::UnitX()))
      .toRotationMatrix();
}

Spherical cartesian_to_spherical(const Vec3& v) {
  const double horizontal = std::hypot(v.x(), v.y());
  return {v.norm(), std::atan2(v.y(), v.x()), std::atan2(v.z(), horizontal)};
}

Vec3 spherical_to_cartesian(const Spherical& s) {
  const double ce = std::cos(s.el);
  return {s.range * ce * std::cos(s.az), s.range * ce * std::sin(s.az), s.range * std::sin(s.el)};
}

double compass_bearing(const Vec3& enu) {
  return wrap_2pi(std::atan2(enu.x(), enu.y()));
}

}  // namespace skytrack::geo
