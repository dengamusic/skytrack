#include "skytrack/rng.hpp"

#include <algorithm>
#include <cmath>

namespace skytrack {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

}  // namespace

double Rng::uniform() {
  // Top 53 bits -> a double in [0, 1) with every value equally likely.
  return static_cast<double>(engine_() >> 11) * 0x1.0p-53;
}

double Rng::uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

double Rng::normal() {
  // Box-Muller. Each call pair uses two uniforms and returns two normals.
  if (has_spare_normal_) {
    has_spare_normal_ = false;
    return spare_normal_;
  }
  const double u1 = 1.0 - uniform();  // (0, 1], so log() is finite
  const double u2 = uniform();
  const double r = std::sqrt(-2.0 * std::log(u1));
  spare_normal_ = r * std::sin(kTwoPi * u2);
  has_spare_normal_ = true;
  return r * std::cos(kTwoPi * u2);
}

double Rng::normal(double mean, double stddev) { return mean + stddev * normal(); }

double Rng::exponential() { return -std::log(1.0 - uniform()); }

bool Rng::bernoulli(double p) { return uniform() < p; }

int Rng::poisson(double mean) {
  if (mean <= 0.0) return 0;
  if (mean < 30.0) {
    // Knuth: count uniforms until their product drops below exp(-mean).
    const double limit = std::exp(-mean);
    int k = 0;
    double product = uniform();
    while (product > limit) {
      ++k;
      product *= uniform();
    }
    return k;
  }
  // Large means: the normal approximation is accurate enough here.
  return std::max(0, static_cast<int>(std::lround(mean + std::sqrt(mean) * normal())));
}

std::uint64_t fnv1a64(std::string_view s) {
  std::uint64_t h = 1469598103934665603ull;
  for (const char c : s) {
    h ^= static_cast<unsigned char>(c);
    h *= 1099511628211ull;
  }
  return h;
}

std::uint64_t mix_seed(std::uint64_t a, std::uint64_t b) { return splitmix64(a ^ splitmix64(b)); }

}  // namespace skytrack
