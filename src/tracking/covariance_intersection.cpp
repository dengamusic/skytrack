#include "skytrack/tracking/covariance_intersection.hpp"

#include <cmath>
#include <limits>

namespace skytrack {

Intersection covariance_intersection(const Gaussian& a, const Gaussian& b, double weight) {
  const Mat6 Ia = a.P.inverse();
  const Mat6 Ib = b.P.inverse();
  Intersection out;
  out.weight = weight;
  out.fused.P = (weight * Ia + (1.0 - weight) * Ib).inverse();
  out.fused.P = 0.5 * (out.fused.P + out.fused.P.transpose()).eval();
  out.gain_a = weight * out.fused.P * Ia;
  out.gain_b = (1.0 - weight) * out.fused.P * Ib;
  out.fused.x = out.gain_a * a.x + out.gain_b * b.x;
  return out;
}

Intersection covariance_intersection(const Gaussian& a, const Gaussian& b) {
  // log det P(w) is convex in w, so a golden-section search finds the minimum.
  const auto cost = [&](double w) {
    const Mat6 info = w * a.P.inverse() + (1.0 - w) * b.P.inverse();
    const double det = info.determinant();
    return det > 0.0 ? -std::log(det) : std::numeric_limits<double>::infinity();
  };
  constexpr double kRatio = 0.6180339887498949;
  double lo = 0.0;
  double hi = 1.0;
  double c = hi - kRatio * (hi - lo);
  double d = lo + kRatio * (hi - lo);
  double fc = cost(c);
  double fd = cost(d);
  for (int i = 0; i < 40; ++i) {  // 0.618^40 ~ 4e-9 of the interval
    if (fc < fd) {
      hi = d;
      d = c;
      fd = fc;
      c = hi - kRatio * (hi - lo);
      fc = cost(c);
    } else {
      lo = c;
      c = d;
      fc = fd;
      d = lo + kRatio * (hi - lo);
      fd = cost(d);
    }
  }
  double w = 0.5 * (lo + hi);
  // When every weight is as tight (equal covariances), average: of all the
  // equally consistent results it has the smallest actual error.
  if (cost(0.5) <= cost(w) + 1e-12 * std::abs(cost(w))) w = 0.5;
  // The ends (one estimate alone) are candidates too.
  if (cost(0.0) < cost(w)) w = 0.0;
  if (cost(1.0) < cost(w)) w = 1.0;
  return covariance_intersection(a, b, w);
}

}  // namespace skytrack
