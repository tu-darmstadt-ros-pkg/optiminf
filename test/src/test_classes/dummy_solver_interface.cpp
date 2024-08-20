#include "optiminf/test/dummy_solver_interface.h"

namespace optiminf
{
namespace test
{
void DummySolverInterface::setupSolver() {}

void DummySolverInterface::prepareSolver() {}

SolverResultStatus DummySolverInterface::runSolverImpl()
{
  bool verbose = getParameterInterfacePtr()->getParam<int>("solver/runtime_print_level") > 0;

  this->updateCostValue(true);

  if (verbose)
  {
    std::cout << "Cost: " << getNlpDataPtr()->total_cost_ << std::endl;
  }
  for (int i = 1; i < 20; i++)
  {
    getNlpDataPtr()->optimization_variables_ -= 0.1 * getNlpDataPtr()->cost_gradient_.transpose();

    this->updateConstraintValues(true);
    this->updateConstraintJacobian(false);
    this->updateCostValue(false);
    this->updateCostGradient(false);

    if (verbose)
    {
      std::cout << "-----------Iteration " << i << " -----------" << std::endl;
      std::cout << "Cost: " << getNlpDataPtr()->total_cost_ << std::endl;
      std::cout << "Cost Gradient: " << std::endl << getNlpDataPtr()->cost_gradient_ << std::endl;
      std::cout << "Opt Var: " << std::endl << getNlpDataPtr()->optimization_variables_ << std::endl;
      std::cout << "Constraint Values: " << std::endl << getNlpDataPtr()->constraint_values_ << std::endl;
      std::cout << "Constraint Jacobian: " << std::endl << getNlpDataPtr()->constraint_jacobian_.toDense() << std::endl;
    }
  }
  return SolverResultStatus::OptimalSolution;
}

void DummySolverInterface::stopSolver() {}
}  // namespace test
}  // namespace optiminf
