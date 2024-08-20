#pragma once

#include "Eigen/Core"

#include "optiminf/data_structs/calculation_result.h"

namespace optiminf
{
/**
 * @brief The NlpData struct contains the neccessray data with which the initialization code, the solver interface and the exporter interacts
 */
struct NlpData
{
  using Ptr = std::shared_ptr<NlpData>;
  using ConstPtr = std::shared_ptr<NlpData>;

  bool is_optimization_running_ = false; /**< @brief this flag is set to true while the optimization is running. */

  Eigen::VectorXd parameters_;
  Eigen::VectorXd optimization_variables_;

  CalculationResult calculation_cache_;

  Eigen::VectorXd constraint_values_;
  Eigen::VectorXd upper_constraint_bounds_;
  Eigen::VectorXd lower_constraint_bounds_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> constraint_jacobian_;

  double total_cost_ = 0;
  Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> cost_gradient_;
};
}  // namespace optiminf
