#pragma once

#include <Eigen/Dense>

/// Coordinate frames and conversions.
///
/// World frame: East-North-Up (ENU) in metres, a flat tangent plane anchored
/// at the scenario origin. Positions broadcast by aircraft and drones are
/// WGS84 latitude/longitude/height and go through ECEF to reach ENU.
///
/// Sensor body frame: x along the boresight, y to the left, z up. Angles
/// measured in a body frame are counter-clockwise: az = atan2(y, x), so a
/// target to the right of the boresight has a negative azimuth.
///
/// Mount headings are compass bearings (0 = north, clockwise positive),
/// because that is how a sensor's orientation is surveyed in the field.
namespace skytrack::geo {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

inline constexpr double kPi = 3.14159265358979323846;

constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / kPi; }

/// Wraps an angle to (-pi, pi].
double wrap_pi(double angle_rad);
/// Wraps an angle to [0, 2*pi).
double wrap_2pi(double angle_rad);

// WGS84 ellipsoid.
inline constexpr double kWgs84A = 6378137.0;
inline constexpr double kWgs84F = 1.0 / 298.257223563;
inline constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

struct Geodetic {
  double lat_rad = 0.0;
  double lon_rad = 0.0;
  double alt_m = 0.0;  // height above the WGS84 ellipsoid
};

Vec3 geodetic_to_ecef(const Geodetic& g);
Geodetic ecef_to_geodetic(const Vec3& ecef);

/// East-North-Up tangent plane anchored at a geodetic origin.
class LocalFrame {
 public:
  LocalFrame() : LocalFrame(Geodetic{}) {}
  explicit LocalFrame(const Geodetic& origin);

  const Geodetic& origin() const { return origin_; }
  Vec3 ecef_to_enu(const Vec3& ecef) const;
  Vec3 enu_to_ecef(const Vec3& enu) const;
  Vec3 geodetic_to_enu(const Geodetic& g) const;
  Geodetic enu_to_geodetic(const Vec3& enu) const;
  /// Rotation taking ECEF vectors (velocities, offsets) into ENU.
  const Mat3& R_enu_from_ecef() const { return R_enu_from_ecef_; }

 private:
  Geodetic origin_;
  Vec3 origin_ecef_;
  Mat3 R_enu_from_ecef_;
};

/// Orientation of a sensor body frame in ENU. heading is a compass bearing of
/// the boresight, pitch tilts the boresight up, roll turns about the boresight.
Mat3 rotation_world_from_body(double heading_rad, double pitch_rad, double roll_rad);

struct Spherical {
  double range = 0.0;
  double az = 0.0;  // atan2(y, x): counter-clockwise
  double el = 0.0;  // atan2(z, horizontal range)
};

Spherical cartesian_to_spherical(const Vec3& v);
Vec3 spherical_to_cartesian(const Spherical& s);

/// Compass bearing of an ENU vector: 0 = north, pi/2 = east, in [0, 2*pi).
double compass_bearing(const Vec3& enu);

}  // namespace skytrack::geo
