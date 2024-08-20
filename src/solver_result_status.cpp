#include "optiminf/solver_interface/solver_result_status.h"

namespace optiminf
{
bool areConstraintsSatisfiedByResult(SolverResultStatus solver_result_status) { return static_cast<int>(solver_result_status) >= 0; }

std::string solverResultStatusToString(SolverResultStatus solver_result_status)
{
  switch (solver_result_status)
  {
    case SolverResultStatus::OptimalSolution:
      return "Optimal solution found";
    case SolverResultStatus::AcceptableSolution:
      return "Acceptable solution found";
    case SolverResultStatus::FeasibleSolution:
      return "Feasible solution found";
    case SolverResultStatus::FeasibleSolutionDespiteError:
      return "Feasible solution was reached despite an error declared as acceptable occured.";
    case SolverResultStatus::FeasibleSolutionIterationLimitReached:
      return "Feasible solution found after iteration limit reached.";
    case SolverResultStatus::FeasibleSoultionTimeLimitReached:
      return "Feasible solution found after time limit reached.";
    case SolverResultStatus::IterationLimitReached:
      return "Maximum number of iterations reached";
    case SolverResultStatus::TimeLimitReached:
      return "Maximum time allowed reached";
    case SolverResultStatus::NotEnoughDegreesOfFreedom:
      return "Not enough degrees of freedom to solve the problem";
    case SolverResultStatus::InfeasibleProblem:
      return "Problem seem to be infeasible";
    case SolverResultStatus::SolverCrashed:
      return "The solver crashed while trying to solve the problem";
    case SolverResultStatus::InvalidNumberDetected:
      return "An invalid number occured while trying to solve the problem";
    case SolverResultStatus::InvalidOption:
      return "An invalid option was used for the solver";
    case SolverResultStatus::InvalidProblemDefinition:
      return "An invalid option was used for the solver";
    case SolverResultStatus::UnspecifiedError:
      return "An unspecified error occured";
    case SolverResultStatus::Aborted:
      return "The solver was aborted";
    default:
      return "No message was specified for the given status";
  }
}

}  // namespace optiminf
