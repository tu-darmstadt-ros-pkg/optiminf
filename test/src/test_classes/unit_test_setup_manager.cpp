#include "optiminf/test/unit_test_setup_manager.h"

#include "optiminf/solver_interface/ipopt/ipopt_interface.h"

#include "optiminf/test/simple_preparation_manager.h"
#include "optiminf/test/simple_calculation_cache.h"
#include "optiminf/test/simple_cost.h"
#include "optiminf/test/simple_constraint.h"
#include "optiminf/test/dummy_solver_interface.h"

namespace optiminf
{
namespace test
{
UnitTestSetupManager::UnitTestSetupManager(bool add_cost, bool add_constraint)
{
  add_cost_ = add_cost;
  add_constraint_ = add_constraint;
}

const ManagerContainer::Ptr& UnitTestSetupManager::getManagerContainerPtrForTest() const { return getManagerContainerPtr(); }

const NlpData::Ptr& UnitTestSetupManager::getNlpDataPtrForTest() const { return getNlpDataPtr(); }

void UnitTestSetupManager::instantiatePreparationManager()
{
  getManagerContainerPtr()->preparation_manager_ptr_ = std::make_shared<SimplePreparationManager>(getManagerContainerPtr(), getNlpDataPtr());
}

void UnitTestSetupManager::instantiateCalculationCache()
{
  getManagerContainerPtr()->calculation_cache_manager_ptr_ = std::make_shared<SimpleCalculationCache>(getManagerContainerPtr(), getNlpDataPtr());
}

void UnitTestSetupManager::instantiateCosts()
{
  if (add_cost_)
  {
    optiminf::CostBase::Ptr cost_ptr = std::make_shared<SimpleCost>();
    getManagerContainerPtr()->cost_manager_ptr_->addCost(cost_ptr);
  }
}

void UnitTestSetupManager::instantiateConstraints()
{
  if (add_constraint_)
  {
    optiminf::ConstraintBase::Ptr constraint_ptr = std::make_shared<SimpleConstraint>();
    getManagerContainerPtr()->constraint_manager_ptr_->addConstraint(constraint_ptr);
  }
}

void UnitTestSetupManager::instantiateSolverInterface()
{
  bool use_dummy_solver = false;
  getManagerContainerPtr()->parameter_interface_ptr_->getParam("solver/use_dummy_solver", use_dummy_solver);

  if (use_dummy_solver)
  {
    getManagerContainerPtr()->solver_interface_ptr_ = std::make_shared<DummySolverInterface>();
  }
  else
  {
    getManagerContainerPtr()->solver_interface_ptr_ = std::make_shared<IpoptInterface>();
  }
}
}  // namespace test
}  // namespace optiminf
