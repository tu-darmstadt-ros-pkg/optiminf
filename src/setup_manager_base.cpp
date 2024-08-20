#include "paraminf/parameter_interface.h"

#include "optiminf/setup/setup_manager_base.h"

namespace optiminf
{
void SetupManagerBase::setupNlp(const ManagerContainer::Ptr manager_container_ptr, const NlpData::Ptr nlp_data_ptr)
{
  manager_container_ptr_ = manager_container_ptr;

  nlp_data_ptr_ = nlp_data_ptr;

  /* instantiate the cost and constraint manager */
  manager_container_ptr_->constraint_manager_ptr_ = std::make_shared<ConstraintManager>(nlp_data_ptr_, manager_container_ptr->parameter_interface_ptr_);
  manager_container_ptr_->cost_manager_ptr_ = std::make_shared<CostManager>(nlp_data_ptr_, manager_container_ptr->parameter_interface_ptr_);

  /* instantiate configuration manager */
  instantiatePreparationManager();

  /* instantiate the calculation cache constraints */
  instantiateCalculationCache();

  /* instantiate the cost and constraints */
  instantiateCosts();
  instantiateConstraints();

  /* instantiate solver interface */
  instantiateSolverInterface();

  /* setup calculation cache */
  manager_container_ptr_->calculation_cache_manager_ptr_->setupCalculationCache();

  /* setup costs and constraints */
  manager_container_ptr_->cost_manager_ptr_->setupCosts();
  manager_container_ptr_->constraint_manager_ptr_->setupConstraints();

  /* setup solver interface */
  manager_container_ptr_->solver_interface_ptr_->setupSolverInterface(manager_container_ptr_, nlp_data_ptr_);
}

const ManagerContainer::Ptr& SetupManagerBase::getManagerContainerPtr() const { return manager_container_ptr_; }

const NlpData::Ptr& SetupManagerBase::getNlpDataPtr() const { return nlp_data_ptr_; }

}  // namespace optiminf
