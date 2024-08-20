#include "optiminf/preparation/preparation_manager_base.h"

namespace optiminf
{
PreparationManagerBase::PreparationManagerBase(const ManagerContainer::Ptr manager_container_ptr, const NlpData::Ptr nlp_data_ptr)
  : manager_container_ptr_(manager_container_ptr)
  , nlp_data_ptr_(nlp_data_ptr)
{}

void PreparationManagerBase::prepareNlp()
{
  initializeOptimizationVariables();
  manager_container_ptr_->calculation_cache_manager_ptr_->prepareCalculationCache();
  manager_container_ptr_->cost_manager_ptr_->prepareCosts();
  manager_container_ptr_->constraint_manager_ptr_->prepareConstraints();
}

const std::shared_ptr<ManagerContainer>& PreparationManagerBase::getManagerContainerPtr() const { return manager_container_ptr_; }

const NlpData::Ptr& PreparationManagerBase::getNlpDataPtr() const { return nlp_data_ptr_; }

}  // namespace optiminf
