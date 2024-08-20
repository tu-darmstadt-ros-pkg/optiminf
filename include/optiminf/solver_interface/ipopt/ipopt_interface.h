#pragma once
// this is necessary due to https://github.com/casadi/casadi/issues/1010
#define HAVE_CSTDDEF
#include <coin/IpTNLP.hpp>
#undef HAVE_CSTDDEF

#include <coin/IpIpoptApplication.hpp>
#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/manager_container.h"

namespace optiminf
{
class IpoptInterface
  : public SolverInterfaceBase
  , public Ipopt::TNLP
{
public:
  using Index = Ipopt::Index;
  using Number = Ipopt::Number;

  void prepareSolver() final;

  SolverResultStatus runSolverImpl() final;

  void stopSolver() final;

  void setupSolver() final;

  void updateOptimizationVariables(const Number* x, bool new_x);

  bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style) final;

  bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u) final;

  bool get_starting_point(Index n, bool init_x, Number* x, bool init_z, Number* z_L, Number* z_U, Index m, bool init_lambda, Number* lambda) final;

  bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) final;

  bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) final;

  bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) final;

  bool eval_jac_g(Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values) final;

  void finalize_solution(Ipopt::SolverReturn status, Index n, const Number* x, const Number* z_L, const Number* z_U, Index m, const Number* g, const Number* lambda,
                         Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq) final;

private:
  Ipopt::SmartPtr<IpoptInterface> ipopt_nlp_;
  std::shared_ptr<Ipopt::IpoptApplication> app_;
  Ipopt::ApplicationReturnStatus status_ = Ipopt::ApplicationReturnStatus::Solve_Succeeded;

  const static std::string LINEAR_SOLVER_PARAM;
  const static std::string RESULT_TOLERANCE_PARAM;
  const static std::string NLP_SCALING_METHOD_PARAM;

  const static std::string RUNTIME_PRINT_LEVEL_PARAM;
  const static std::string RUNTIME_TIMING_STATISTICS_PARAM;
  const static std::string MAXIMUM_ITERATIONS_PARAM;

  const static std::string USE_HESSIAN_APPROXIMATION_PARAM;

  const static std::string OUTPUT_FILE_PATH_PARAM;

  const static std::string PERFORM_DERIVATIVE_TEST_PARAM;
};
}  // namespace optiminf
