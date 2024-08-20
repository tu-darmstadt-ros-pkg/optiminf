#pragma once

#include "paraminf/parameter_interface.h"

#include "optiminf/preparation/preparation_manager_base.h"
#include "optiminf/costs/cost_manager.h"
#include "optiminf/constraints/constraint_manager.h"
#include "optiminf/calculation_cache/calculation_cache_manager_base.h"
#include "optiminf/solver_interface/solver_interface_base.h"
#include "optiminf/export/exporter_base.h"

namespace optiminf
{
class PreparationManagerBase;
class SolverInterfaceBase;
class CalculationCacheManagerBase;
class ExporterBase;

/**
 * @brief The ManagerContainer struct contains pointer to all manager that are neccessary for solving a problem.
 * @details This struct is intended to simplify the arguments of the constructors s.t. these require only one struct and not many pointers.
 */
struct ManagerContainer
{
  using Ptr = std::shared_ptr<ManagerContainer>;
  using ConstPtr = std::shared_ptr<const ManagerContainer>;

  paraminf::ParameterInterface::Ptr parameter_interface_ptr_;

  std::shared_ptr<PreparationManagerBase> preparation_manager_ptr_;

  std::shared_ptr<CalculationCacheManagerBase> calculation_cache_manager_ptr_;

  ConstraintManager::Ptr constraint_manager_ptr_;
  CostManager::Ptr cost_manager_ptr_;

  std::shared_ptr<SolverInterfaceBase> solver_interface_ptr_;

  std::vector<std::shared_ptr<ExporterBase>> exporter_ptr_vec_;
};
}  // namespace optiminf
