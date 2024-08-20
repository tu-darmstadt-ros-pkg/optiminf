#include "optiminf/solver_interface/ipopt/ipopt_interface.h"

#include "optiminf/util/file_system_util.h"
#include "optiminf/error_handling/optiminf_exception.h"

namespace optiminf
{
const std::string IpoptInterface::LINEAR_SOLVER_PARAM = "solver/linear_solver";
const std::string IpoptInterface::RESULT_TOLERANCE_PARAM = "solver/result_tolerance";
const std::string IpoptInterface::NLP_SCALING_METHOD_PARAM = "solver/nlp_scaling_method";

const std::string IpoptInterface::RUNTIME_PRINT_LEVEL_PARAM = "solver/runtime_print_level";
const std::string IpoptInterface::RUNTIME_TIMING_STATISTICS_PARAM = "solver/runtime_timing_statistics";

const std::string IpoptInterface::MAXIMUM_ITERATIONS_PARAM = "solver/maximum_iterations";

const std::string IpoptInterface::USE_HESSIAN_APPROXIMATION_PARAM = "solver/use_hessian_approximation";

const std::string IpoptInterface::OUTPUT_FILE_PATH_PARAM = "solver/output_file_path";

const std::string IpoptInterface::PERFORM_DERIVATIVE_TEST_PARAM = "solver/perform_derivative_test";

void IpoptInterface::prepareSolver() {}

SolverResultStatus IpoptInterface::runSolverImpl()
{
  try
  {
    status_ = app_->OptimizeTNLP(ipopt_nlp_);
  }
  catch (...)
  {
    return SolverResultStatus::SolverCrashed;
  }

  switch (status_)
  {
    case Ipopt::ApplicationReturnStatus::Solve_Succeeded:
      return SolverResultStatus::OptimalSolution;
    case Ipopt::ApplicationReturnStatus::Solved_To_Acceptable_Level:
      return SolverResultStatus::AcceptableSolution;
    case Ipopt::ApplicationReturnStatus::Feasible_Point_Found:
      return SolverResultStatus::FeasibleSolution;
    case Ipopt::ApplicationReturnStatus::Infeasible_Problem_Detected:
      return SolverResultStatus::InfeasibleProblem;
    case Ipopt::ApplicationReturnStatus::Invalid_Number_Detected:
      return SolverResultStatus::InvalidNumberDetected;
    case Ipopt::ApplicationReturnStatus::Maximum_Iterations_Exceeded:
      return SolverResultStatus::IterationLimitReached;
    case Ipopt::ApplicationReturnStatus::Maximum_CpuTime_Exceeded:
      return SolverResultStatus::TimeLimitReached;
    case Ipopt::ApplicationReturnStatus::User_Requested_Stop:
      return SolverResultStatus::Aborted;
    case Ipopt::ApplicationReturnStatus::Unrecoverable_Exception:
      return SolverResultStatus::SolverCrashed;
    case Ipopt::ApplicationReturnStatus::NonIpopt_Exception_Thrown:
      return SolverResultStatus::SolverCrashed;
    case Ipopt::ApplicationReturnStatus::Invalid_Option:
      return SolverResultStatus::InvalidOption;
    case Ipopt::ApplicationReturnStatus::Invalid_Problem_Definition:
      return SolverResultStatus::InvalidProblemDefinition;
    case Ipopt::ApplicationReturnStatus::Not_Enough_Degrees_Of_Freedom:
      return SolverResultStatus::NotEnoughDegreesOfFreedom;
    default:
      return SolverResultStatus::UnspecifiedError;
  }
}

void IpoptInterface::stopSolver() {}

void IpoptInterface::setupSolver()
{
  ipopt_nlp_ = this;
  app_ = std::make_shared<Ipopt::IpoptApplication>();

  // set all available options
  std::string linear_solver;
  if (getParameterInterfacePtr()->getParam(LINEAR_SOLVER_PARAM, linear_solver))
  {
    app_->Options()->SetStringValue("linear_solver", linear_solver);
  }
  double tol = 0.0;
  if (getParameterInterfacePtr()->getParam(RESULT_TOLERANCE_PARAM, tol))
  {
    app_->Options()->SetNumericValue("tol", tol);
  }
  std::string scaling_method;
  if (getParameterInterfacePtr()->getParam(NLP_SCALING_METHOD_PARAM, scaling_method))
  {
    app_->Options()->SetStringValue("nlp_scaling_method", scaling_method);
  }

  int print_level = 0;
  if (getParameterInterfacePtr()->getParam(RUNTIME_PRINT_LEVEL_PARAM, print_level))
  {
    app_->Options()->SetIntegerValue("print_level", print_level);
  }
  if (print_level < 5)
  {
    /* deactivate ipopt header printout */
    app_->Options()->SetStringValue("sb", "yes");
  }

  bool timing_statistics = false;
  if (getParameterInterfacePtr()->getParam(RUNTIME_TIMING_STATISTICS_PARAM, timing_statistics))
  {
    if (timing_statistics)
    {
      app_->Options()->SetStringValue("print_timing_statistics", "yes");
    }
    app_->Options()->SetStringValue("print_timing_statistics", "no");
  }

  int max_iterations = 0;
  if (getParameterInterfacePtr()->getParam(MAXIMUM_ITERATIONS_PARAM, max_iterations))
  {
    app_->Options()->SetIntegerValue("max_iter", max_iterations);
  }

  std::string use_hessian_approx;
  if (getParameterInterfacePtr()->getParam(USE_HESSIAN_APPROXIMATION_PARAM, use_hessian_approx))
  {
    app_->Options()->SetStringValue("hessian_approximation", use_hessian_approx);
  }

  std::string output_file_path;
  if (getParameterInterfacePtr()->getParam(OUTPUT_FILE_PATH_PARAM, output_file_path))
  {
    // create directory if it does not exist yet
    FileSystemUtil::createDirectoryIfNonexistent(FileSystemUtil::getDirectoryOfFile(output_file_path));
    app_->Options()->SetStringValue("output_file", output_file_path);
  }

  bool perform_derivative_test = false;
  getParameterInterfacePtr()->getParam(PERFORM_DERIVATIVE_TEST_PARAM, perform_derivative_test);
  if (perform_derivative_test)
  {
    app_->Options()->SetStringValue("derivative_test", "first-order");
  }
  else
  {
    app_->Options()->SetStringValue("derivative_test", "none");
  }

  status_ = app_->Initialize();
  if (status_ != Ipopt::Solve_Succeeded)
  {
    throw SetupError("IpoptInterface: An error during initialization of IPOPT occurred!");
  }
}

void IpoptInterface::updateOptimizationVariables(const Number* x, bool new_x)
{
  if (new_x)
  {
    memcpy(getNlpDataPtr()->optimization_variables_.data(), x, getNlpDataPtr()->optimization_variables_.size() * sizeof(Number));
  }
}

bool IpoptInterface::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& /*nnz_h_lag*/, IndexStyleEnum& index_style)
{
  n = getNlpDataPtr()->optimization_variables_.size();
  m = getNlpDataPtr()->constraint_values_.size();
  updateConstraintJacobian(true);
  nnz_jac_g = getNlpDataPtr()->constraint_jacobian_.nonZeros();
  index_style = TNLP::C_STYLE;
  return true;
}

bool IpoptInterface::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u)
{
  assert(n == getNlpDataPtr()->optimization_variables_.size());
  assert(m == getNlpDataPtr()->constraint_values_.size());
  for (size_t i = 0; i < n; i++)
  {
    app_->Options()->GetNumericValue("nlp_lower_bound_inf", x_l[i], "");  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic): ignore clang-warnings for pointer arithmetic
    app_->Options()->GetNumericValue("nlp_upper_bound_inf", x_u[i], "");  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic): ignore clang-warnings for pointer arithmetic
  }
  memcpy(g_l, getNlpDataPtr()->lower_constraint_bounds_.data(), getNlpDataPtr()->lower_constraint_bounds_.size() * sizeof(Number));
  memcpy(g_u, getNlpDataPtr()->upper_constraint_bounds_.data(), getNlpDataPtr()->upper_constraint_bounds_.size() * sizeof(Number));
  return true;
}

bool IpoptInterface::get_starting_point(Index n, bool init_x, Number* x, bool init_z, Number* /*z_L*/, Number* /*z_U*/, Index m, bool init_lambda, Number* /*lambda*/)
{
  assert(n == getNlpDataPtr()->optimization_variables_.size());
  assert(m == getNlpDataPtr()->constraint_values_.size());
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);
  memcpy(x, getNlpDataPtr()->optimization_variables_.data(), getNlpDataPtr()->optimization_variables_.size() * sizeof(Number));
  return true;
}

bool IpoptInterface::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  assert(n == getNlpDataPtr()->optimization_variables_.size());
  updateOptimizationVariables(x, new_x);
  updateCostValue(new_x);
  obj_value = getNlpDataPtr()->total_cost_;
  return true;
}

bool IpoptInterface::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  assert(n == getNlpDataPtr()->optimization_variables_.size());
  updateOptimizationVariables(x, new_x);
  updateCostGradient(new_x);
  memcpy(grad_f, getNlpDataPtr()->cost_gradient_.data(), getNlpDataPtr()->cost_gradient_.size() * sizeof(Number));
  return true;
}

bool IpoptInterface::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  assert(n == getNlpDataPtr()->optimization_variables_.size());
  assert(m == getNlpDataPtr()->constraint_values_.size());
  updateOptimizationVariables(x, new_x);
  updateConstraintValues(new_x);
  memcpy(g, getNlpDataPtr()->constraint_values_.data(), getNlpDataPtr()->constraint_values_.size() * sizeof(Number));
  return true;
}

bool IpoptInterface::eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
  assert(n == getNlpDataPtr()->optimization_variables_.size());
  assert(m == getNlpDataPtr()->constraint_values_.size());
  if (values == nullptr)
  {
    nele_jac = 0;
    for (int k = 0; k < getNlpDataPtr()->constraint_jacobian_.outerSize(); ++k)
    {
      for (Eigen::SparseMatrix<Number, Eigen::RowMajor>::InnerIterator it(getNlpDataPtr()->constraint_jacobian_, k); it; ++it)
      {
        iRow[nele_jac] = it.row();  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic): ignore clang-warnings for pointer arithmetic
        jCol[nele_jac] = it.col();  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic): ignore clang-warnings for pointer arithmetic
        nele_jac++;
      }
    }
    assert(nele_jac == getNlpDataPtr()->constraint_jacobian_.nonZeros());
  }
  else
  {
    assert(nele_jac == getNlpDataPtr()->constraint_jacobian_.nonZeros());
    updateOptimizationVariables(x, new_x);
    updateConstraintJacobian(new_x);
    memcpy(values, getNlpDataPtr()->constraint_jacobian_.valuePtr(), getNlpDataPtr()->constraint_jacobian_.nonZeros() * sizeof(Number));
  }
  return true;
}

void IpoptInterface::finalize_solution(Ipopt::SolverReturn /*status*/, Index n, const Number* x, const Number* /*z_L*/, const Number* /*z_U*/, Index m, const Number* /*g*/,
                                       const Number* /*lambda*/, Number /*obj_value*/, const Ipopt::IpoptData* /*ip_data*/, Ipopt::IpoptCalculatedQuantities* /*ip_cq*/)
{
  assert(n == getNlpDataPtr()->optimization_variables_.size());
  assert(m == getNlpDataPtr()->constraint_values_.size());
  updateOptimizationVariables(x, true);
}

}  // namespace optiminf
