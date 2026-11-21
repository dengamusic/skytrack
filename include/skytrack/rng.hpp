#pragma once

#include <cstdint>
#include <random>
#include <string_view>

namespace skytrack {

/// Deterministic random source.
///
/// The engine is std::mt19937_64, whose output the C++ standard fixes. The
/// distributions are written here instead of using std::normal_distribution
/// and friends, because those differ between libc++ and libstdc++: with them
/// the same seed would give different runs on macOS and on Linux CI.
class Rng {
 public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}

  double uniform();                      // [0, 1)
  double uniform(double lo, double hi);  // [lo, hi)
  double normal();                       // N(0, 1)
  double normal(double mean, double stddev);
  double exponential();                  // mean 1
  bool bernoulli(double p);
  int poisson(double mean);

 private:
  std::mt19937_64 engine_;
  bool has_spare_normal_ = false;
  double spare_normal_ = 0.0;
};

/// Stable 64-bit string hash (std::hash is not stable across platforms).
std::uint64_t fnv1a64(std::string_view s);
/// Combines two values into a well-mixed seed.
std::uint64_t mix_seed(std::uint64_t a, std::uint64_t b);

}  // namespace skytrack
