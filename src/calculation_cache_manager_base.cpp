#include "optiminf/calculation_cache/calculation_cache_manager_base.h"

#include "optiminf/util/parameter_naming_util.h"
#include "optiminf/util/eigen_util.h"

namespace optiminf
{
CalculationCacheManagerBase::CalculationCacheManagerBase(const std::shared_ptr<const ManagerContainer> manager_container_ptr, const NlpData::Ptr nlp_data_ptr)
  : manager_container_ptr_(std::move(manager_container_ptr))
  , nlp_data_ptr_(std::move(nlp_data_ptr))
{
  manager_container_ptr->parameter_interface_ptr_->getParam(ParameterNamingUtil::CHECK_FOR_NON_FINITE_VALUES_PARAM_NAME, check_for_non_finite_values_);
}

void CalculationCacheManagerBase::updateCacheStatus(const bool new_input_values)
{
  calculation_cache_values_valid_ &= !new_input_values;
  calculation_cache_jacobian_valid_ &= !new_input_values;

  // check values to be finite if option is set
  if (check_for_non_finite_values_)
  {
    EigenUtil::checkVectorForNonFiniteValue(nlp_data_ptr_->optimization_variables_, "Non finite value for optimization variable.");
  }
}

void CalculationCacheManagerBase::ensureValidCacheValues()
{
  if (!calculation_cache_values_valid_)
  {
    updateValues();
    calculation_cache_values_valid_ = true;

    // check values to be finite if option is set
    if (check_for_non_finite_values_)
    {
      EigenUtil::checkVectorForNonFiniteValue(nlp_data_ptr_->calculation_cache_.values_, "Non finite value in calculation cache.");
    }
  }
}

void CalculationCacheManagerBase::ensureValidCacheJacobian()
{
  if (!calculation_cache_jacobian_valid_)
  {
    updateJacobian();
    calculation_cache_jacobian_valid_ = true;

    // check values to be finite if option is set
    if (check_for_non_finite_values_)
    {
      EigenUtil::checkSparseMatrixForNonFiniteValue(nlp_data_ptr_->calculation_cache_.jacobian_map_, "Value of jacobian of calculation cache is not finite.");
    }
  }
}

const Eigen::VectorXd& CalculationCacheManagerBase::getOptVarVec() const { return nlp_data_ptr_->optimization_variables_; }

const Eigen::VectorXd& CalculationCacheManagerBase::getParamVec() const { return nlp_data_ptr_->parameters_; }

CalculationResult& CalculationCacheManagerBase::calculationCacheResult() { return nlp_data_ptr_->calculation_cache_; }

std::shared_ptr<const ManagerContainer> CalculationCacheManagerBase::getManagerContainerPtr() { return manager_container_ptr_; }

}  // namespace optiminf
