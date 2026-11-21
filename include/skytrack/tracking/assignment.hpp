#pragma once

#include <limits>
#include <vector>

#include <Eigen/Dense>

namespace skytrack {

/// Marks a pairing that must never be chosen (outside the gate, conflicting identity, ...).
inline constexpr double kForbidden = std::numeric_limits<double>::infinity();

/// Minimum-cost assignment of rows to columns (Hungarian algorithm with
/// potentials, O(n^2 m)). Works for any shape. Returns, for each row, its
/// column, or -1 if the row got none: either there are more rows than
/// columns, or every column left for it is kForbidden.
std::vector<int> solve_assignment(const Eigen::MatrixXd& cost);

/// Assignment where each row may also stay unassigned, at a cost of
/// unassigned_cost[row]. This is global nearest neighbour: a measurement
/// either updates one track or is left over, whichever is cheaper overall.
std::vector<int> solve_assignment_with_misses(const Eigen::MatrixXd& cost, const Eigen::VectorXd& unassigned_cost);

}  // namespace skytrack
