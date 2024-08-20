#pragma once

#include "optiminf/solver_interface/solver_interface_base.h"

namespace optiminf
{
namespace test
{
/**
 * @brief The DummySolverInterface class
 */
class DummySolverInterface : public optiminf::SolverInterfaceBase
{
  void setupSolver() final;

  void prepareSolver() final;

  SolverResultStatus runSolverImpl() final;

  void stopSolver() final;
};
}  // namespace test
}  // namespace optiminf
