#include <gtest/gtest.h>

#include <cmath>

#include "skytrack/rng.hpp"

using namespace skytrack;

TEST(Rng, SameSeedGivesSameSequence) {
  Rng a(123);
  Rng b(123);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_EQ(a.normal(), b.normal());
    EXPECT_EQ(a.poisson(2.0), b.poisson(2.0));
  }
}

TEST(Rng, UniformStaysInRange) {
  Rng rng(1);
  for (int i = 0; i < 100000; ++i) {
    const double u = rng.uniform();
    ASSERT_GE(u, 0.0);
    ASSERT_LT(u, 1.0);
  }
}

TEST(Rng, NormalHasUnitMoments) {
  Rng rng(2);
  const int n = 200000;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double x = rng.normal();
    sum += x;
    sum_sq += x * x;
  }
  const double mean = sum / n;
  EXPECT_NEAR(mean, 0.0, 0.01);
  EXPECT_NEAR(std::sqrt(sum_sq / n - mean * mean), 1.0, 0.01);
}

TEST(Rng, PoissonMeanAndVariance) {
  Rng rng(3);
  const int n = 100000;
  const double lambda = 2.5;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double k = rng.poisson(lambda);
    sum += k;
    sum_sq += k * k;
  }
  const double mean = sum / n;
  EXPECT_NEAR(mean, lambda, 0.03);
  EXPECT_NEAR(sum_sq / n - mean * mean, lambda, 0.06);
}

TEST(Rng, SeedMixingSeparatesStreams) {
  EXPECT_NE(mix_seed(1, fnv1a64("radar-1")), mix_seed(1, fnv1a64("radar-2")));
  EXPECT_NE(mix_seed(1, fnv1a64("radar-1")), mix_seed(2, fnv1a64("radar-1")));
  EXPECT_EQ(fnv1a64("radar-1"), fnv1a64("radar-1"));
}
