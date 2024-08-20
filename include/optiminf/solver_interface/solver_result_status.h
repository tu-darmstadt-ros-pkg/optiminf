#pragma once

#include <string>

namespace optiminf
{
/**
 * @brief The SolverResultStatus enum zero or positive values indicate that at least the constraints are satisfied
 */
enum class SolverResultStatus
{
  OptimalSolution = 0,
  AcceptableSolution = 1,
  FeasibleSolution = 2,
  FeasibleSolutionIterationLimitReached = 3,
  FeasibleSoultionTimeLimitReached = 4,
  FeasibleSolutionDespiteError = 5,
  IterationLimitReached = -1,
  TimeLimitReached = -2,
  NotEnoughDegreesOfFreedom = -10,
  InfeasibleProblem = -11,
  SolverCrashed = -12,
  InvalidNumberDetected = -13,
  InvalidOption = -14,
  InvalidProblemDefinition = -15,
  UnspecifiedError = -16,
  Aborted = -17
};

/**
 * @brief areConstraintsSatisfiedByResult checks whether the result status indicates that all constraints are fullfilled
 * @param solver_result_status the solver result status that should be checked
 * @return  true if the solution satisfies all constraints
 */
bool areConstraintsSatisfiedByResult(SolverResultStatus solver_result_status);

std::string solverResultStatusToString(SolverResultStatus solver_result_status);

}  // namespace optiminf
