#include "skytrack/tracking/assignment.hpp"

#include <cmath>
#include <stdexcept>

namespace skytrack {
namespace {

/// Hungarian algorithm for rows <= cols and finite costs. Every row is assigned.
std::vector<int> hungarian(const Eigen::MatrixXd& a) {
  const int n = static_cast<int>(a.rows());
  const int m = static_cast<int>(a.cols());
  constexpr double kInf = std::numeric_limits<double>::infinity();
  // 1-based potentials; column 0 is a virtual start column.
  std::vector<double> u(static_cast<std::size_t>(n) + 1, 0.0);
  std::vector<double> v(static_cast<std::size_t>(m) + 1, 0.0);
  std::vector<int> owner(static_cast<std::size_t>(m) + 1, 0);  // row assigned to each column
  std::vector<int> way(static_cast<std::size_t>(m) + 1, 0);
  const auto at = [](auto& vec, int i) -> auto& { return vec[static_cast<std::size_t>(i)]; };

  for (int i = 1; i <= n; ++i) {
    at(owner, 0) = i;
    int j0 = 0;
    std::vector<double> min_slack(static_cast<std::size_t>(m) + 1, kInf);
    std::vector<char> used(static_cast<std::size_t>(m) + 1, 0);
    do {
      at(used, j0) = 1;
      const int i0 = at(owner, j0);
      double delta = kInf;
      int j1 = 0;
      for (int j = 1; j <= m; ++j) {
        if (at(used, j)) continue;
        const double slack = a(i0 - 1, j - 1) - at(u, i0) - at(v, j);
        if (slack < at(min_slack, j)) {
          at(min_slack, j) = slack;
          at(way, j) = j0;
        }
        if (at(min_slack, j) < delta) {
          delta = at(min_slack, j);
          j1 = j;
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (at(used, j)) {
          at(u, at(owner, j)) += delta;
          at(v, j) -= delta;
        } else {
          at(min_slack, j) -= delta;
        }
      }
      j0 = j1;
    } while (at(owner, j0) != 0);
    do {  // flip the augmenting path
      const int j1 = at(way, j0);
      at(owner, j0) = at(owner, j1);
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> row_to_col(static_cast<std::size_t>(n), -1);
  for (int j = 1; j <= m; ++j) {
    if (at(owner, j) != 0) row_to_col[static_cast<std::size_t>(at(owner, j) - 1)] = j - 1;
  }
  return row_to_col;
}

/// A finite stand-in for kForbidden that is dearer than any assignment made
/// only of allowed entries, so the solver uses one only if it has to.
double forbidden_stand_in(const Eigen::MatrixXd& cost) {
  double sum = 1.0;
  for (Eigen::Index i = 0; i < cost.size(); ++i) {
    const double c = cost.data()[i];
    if (std::isnan(c)) throw std::invalid_argument("assignment cost is NaN");
    if (std::isfinite(c)) sum += std::abs(c);
  }
  return 2.0 * sum;
}

}  // namespace

std::vector<int> solve_assignment(const Eigen::MatrixXd& cost) {
  if (cost.rows() == 0) return {};
  if (cost.cols() == 0) return std::vector<int>(static_cast<std::size_t>(cost.rows()), -1);

  const double big = forbidden_stand_in(cost);
  const bool transpose = cost.rows() > cost.cols();
  Eigen::MatrixXd a = transpose ? Eigen::MatrixXd(cost.transpose()) : cost;
  a = a.unaryExpr([big](double c) { return std::isfinite(c) ? c : big; });

  const std::vector<int> solved = hungarian(a);
  std::vector<int> row_to_col(static_cast<std::size_t>(cost.rows()), -1);
  for (std::size_t k = 0; k < solved.size(); ++k) {
    const int col = solved[k];
    if (col < 0) continue;
    const int r = transpose ? col : static_cast<int>(k);
    const int c = transpose ? static_cast<int>(k) : col;
    if (std::isfinite(cost(r, c))) row_to_col[static_cast<std::size_t>(r)] = c;
  }
  return row_to_col;
}

std::vector<int> solve_assignment_with_misses(const Eigen::MatrixXd& cost, const Eigen::VectorXd& unassigned_cost) {
  if (unassigned_cost.size() != cost.rows()) throw std::invalid_argument("need one unassigned cost per row");
  const Eigen::Index n = cost.rows();
  const Eigen::Index m = cost.cols();
  // Extra columns, one per row: "row i stays unassigned", open to row i only.
  Eigen::MatrixXd augmented = Eigen::MatrixXd::Constant(n, m + n, kForbidden);
  augmented.leftCols(m) = cost;
  for (Eigen::Index i = 0; i < n; ++i) augmented(i, m + i) = unassigned_cost[i];

  std::vector<int> row_to_col = solve_assignment(augmented);
  for (int& col : row_to_col) {
    if (col >= m) col = -1;
  }
  return row_to_col;
}

}  // namespace skytrack
