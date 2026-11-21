#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "skytrack/eval/metrics.hpp"
#include "skytrack/rng.hpp"
#include "skytrack/tracking/assignment.hpp"

namespace skytrack {
namespace {

/// Cheapest assignment by trying every injective map of rows into columns
/// (rows <= cols), allowing a row to stay unassigned at `leave` cost.
double brute_force(const Eigen::MatrixXd& cost, double leave) {
  const auto n = static_cast<int>(cost.rows());
  const auto m = static_cast<int>(cost.cols());
  double best = std::numeric_limits<double>::infinity();
  std::vector<int> choice(static_cast<std::size_t>(n), -1);
  std::vector<char> used(static_cast<std::size_t>(m), 0);
  const auto search = [&](auto&& self, int row, double total) -> void {
    if (total >= best) return;
    if (row == n) {
      best = total;
      return;
    }
    self(self, row + 1, total + leave);  // leave this row out
    for (int c = 0; c < m; ++c) {
      if (used[static_cast<std::size_t>(c)] || !std::isfinite(cost(row, c))) continue;
      used[static_cast<std::size_t>(c)] = 1;
      self(self, row + 1, total + cost(row, c));
      used[static_cast<std::size_t>(c)] = 0;
    }
  };
  search(search, 0, 0.0);
  return best;
}

double total_cost(const Eigen::MatrixXd& cost, const std::vector<int>& rows_to_cols, double leave) {
  double total = 0.0;
  for (std::size_t r = 0; r < rows_to_cols.size(); ++r) {
    total += rows_to_cols[r] < 0 ? leave : cost(static_cast<Eigen::Index>(r), rows_to_cols[r]);
  }
  return total;
}

TEST(Assignment, SolvesSmallSquareProblem) {
  Eigen::MatrixXd cost(3, 3);
  cost << 4, 1, 3,  //
      2, 0, 5,      //
      3, 2, 2;
  const auto a = solve_assignment(cost);
  ASSERT_EQ(a.size(), 3u);
  EXPECT_DOUBLE_EQ(total_cost(cost, a, 0.0), 5.0);  // 1 + 2 + 2
}

TEST(Assignment, HandlesRectangularAndEmpty) {
  Eigen::MatrixXd wide(2, 4);
  wide << 9, 1, 9, 9,  //
      9, 9, 9, 2;
  EXPECT_EQ(solve_assignment(wide), (std::vector<int>{1, 3}));

  const Eigen::MatrixXd tall = wide.transpose();
  const auto a = solve_assignment(tall);
  EXPECT_EQ(a[1], 0);
  EXPECT_EQ(a[3], 1);
  EXPECT_EQ(std::count(a.begin(), a.end(), -1), 2);

  EXPECT_TRUE(solve_assignment(Eigen::MatrixXd(0, 3)).empty());
  EXPECT_EQ(solve_assignment(Eigen::MatrixXd(2, 0)), (std::vector<int>{-1, -1}));
}

TEST(Assignment, NeverPicksForbiddenPairs) {
  Eigen::MatrixXd cost(2, 2);
  cost << kForbidden, 1.0,  //
      kForbidden, 2.0;
  const auto a = solve_assignment(cost);
  // Only one row can get column 1; the other has nothing allowed.
  EXPECT_EQ(std::count(a.begin(), a.end(), 1), 1);
  EXPECT_EQ(std::count(a.begin(), a.end(), -1), 1);
  EXPECT_EQ(a[0], 1);  // the cheaper way to use column 1
}

TEST(Assignment, GlobalNearestNeighbourMatchesBruteForce) {
  Rng rng(7);
  for (int trial = 0; trial < 300; ++trial) {
    const auto n = static_cast<Eigen::Index>(1 + trial % 5);
    const auto m = static_cast<Eigen::Index>(1 + (trial / 5) % 5);
    const double leave = 6.0;
    Eigen::MatrixXd cost(n, m);
    for (Eigen::Index i = 0; i < n; ++i) {
      for (Eigen::Index j = 0; j < m; ++j) {
        cost(i, j) = rng.bernoulli(0.3) ? kForbidden : rng.uniform(0.0, 10.0);
      }
    }
    const auto a = solve_assignment_with_misses(cost, Eigen::VectorXd::Constant(n, leave));
    // Valid: each column used at most once, never a forbidden pair.
    std::vector<int> used;
    for (Eigen::Index i = 0; i < n; ++i) {
      const int c = a[static_cast<std::size_t>(i)];
      if (c < 0) continue;
      EXPECT_TRUE(std::isfinite(cost(i, c)));
      EXPECT_EQ(std::count(used.begin(), used.end(), c), 0);
      used.push_back(c);
    }
    EXPECT_NEAR(total_cost(cost, a, leave), brute_force(cost, leave), 1e-9) << "trial " << trial;
  }
}

TEST(Gospa, MatchesHandComputedValues) {
  const double c = 10.0;
  // One target, one estimate 3 m away: just the localisation error.
  EXPECT_NEAR(eval::gospa({Vec3(0, 0, 0)}, {Vec3(3, 0, 0)}, c, 2.0).total, 3.0, 1e-12);
  // A missed target costs c / 2^(1/p).
  const auto missed = eval::gospa({Vec3(0, 0, 0)}, {}, c, 2.0);
  EXPECT_NEAR(missed.total, c / std::sqrt(2.0), 1e-12);
  EXPECT_EQ(missed.missed, 1);
  // Further apart than c: one miss plus one false track, not a match.
  const auto apart = eval::gospa({Vec3(0, 0, 0)}, {Vec3(50, 0, 0)}, c, 2.0);
  EXPECT_NEAR(apart.total, c, 1e-12);
  EXPECT_EQ(apart.matched, 0);
  EXPECT_EQ(apart.false_tracks, 1);
  // Optimal pairing, not greedy: pairing the closest two (1 m) would leave 8 m for the others.
  const auto pairs = eval::gospa({Vec3(0, 0, 0), Vec3(2, 0, 0)}, {Vec3(1, 0, 0), Vec3(-6, 0, 0)}, c, 2.0);
  EXPECT_NEAR(pairs.total, std::sqrt(36.0 + 1.0), 1e-12);  // 0 <-> -6 (6 m), 2 <-> 1 (1 m)
}

}  // namespace
}  // namespace skytrack
