#pragma once

#include "skytrack/tracking/motion.hpp"

namespace skytrack {

/// Covariance intersection (Julier and Uhlmann, 1997): fuses two estimates
/// of one state whose errors are correlated by an unknown amount.
///
///   P^-1 = w Pa^-1 + (1 - w) Pb^-1
///   x    = P (w Pa^-1 xa + (1 - w) Pb^-1 xb)
///
/// The result is consistent (P is at least the error covariance) for every
/// cross-correlation the two could have. That makes it the safe choice when
/// two tracks shared sensors, or two nodes heard the same target, and nobody
/// kept the cross-covariance. The weight w minimises det P (the volume of the
/// uncertainty ellipsoid).
struct Intersection {
  Gaussian fused;
  double weight = 0.5;  // w, the weight on the first estimate
  /// fused error = gain_a * (error of a) + gain_b * (error of b), so anything
  /// correlated with the inputs' errors carries over to the result.
  Mat6 gain_a = Mat6::Zero();
  Mat6 gain_b = Mat6::Zero();
};

Intersection covariance_intersection(const Gaussian& a, const Gaussian& b);

/// The same formula for a given weight (no search).
Intersection covariance_intersection(const Gaussian& a, const Gaussian& b, double weight);

}  // namespace skytrack
