#include "optiminf/costs/cost_manager.h"

#include "optiminf/error_handling/optiminf_exception.h"
#include "optiminf/util/parameter_naming_util.h"

#include <math.h>
namespace optiminf
{
CostManager::CostManager(const NlpData::Ptr nlp_data_ptr, const paraminf::ParameterInterface::Ptr parameter_interface_ptr)
  : nlp_data_ptr_(nlp_data_ptr)
  , parameter_interface_ptr_(parameter_interface_ptr)
{}

void CostManager::setupCosts()
{
  parameter_interface_ptr_->getParam(ParameterNamingUtil::COST_DEBUG_LEVEL_PARAM_NAME, cost_debug_level_);
  parameter_interface_ptr_->getParam(ParameterNamingUtil::CHECK_FOR_NON_FINITE_VALUES_PARAM_NAME, check_for_non_finite_values_);

  for (const auto& cost_ptr : cost_ptr_vec_)
  {
    cost_ptr->setupCost(nlp_data_ptr_, parameter_interface_ptr_);
  }
}

void CostManager::prepareCosts()
{
  for (const auto& cost_ptr : cost_ptr_vec_)
  {
    cost_ptr->prepareCost();
  }

  size_t nr_of_opt_vars = nlp_data_ptr_->optimization_variables_.size();

  nlp_data_ptr_->cost_gradient_.resize(1, nr_of_opt_vars);
}

void CostManager::updateCostValue()
{
  for (const auto& cost_ptr : cost_ptr_vec_)
  {
    cost_ptr->updateValue();
  }

  addUpCosts();
}

void CostManager::updateCostGradient()
{
  for (const auto& cost_ptr : cost_ptr_vec_)
  {
    cost_ptr->updateGradient();
  }
  addUpGradients();
}

void CostManager::addCost(const CostBase::Ptr cost_ptr) { cost_ptr_vec_.push_back(cost_ptr); }

void CostManager::printCostValues() const
{
  std::cout << "Weighted cost values:" << std::endl;
  for (const auto& cost_ptr : cost_ptr_vec_)
  {
    std::cout << "\t" << cost_ptr->getCostName() << ": " << cost_ptr->getWeightedCostValue() << std::endl;
  }
}

void CostManager::addUpCosts()
{
  nlp_data_ptr_->total_cost_ = 0;
  for (const auto& cost_ptr : cost_ptr_vec_)
  {
    double cost_value = cost_ptr->getWeightedCostValue();
    // check cost for having a non finite value if option is set
    if (check_for_non_finite_values_)
    {
      if (!isfinite(cost_value))
      {
        throw NumericError("Value of cost named \"" + cost_ptr->getCostName() + "\" is not finite but has value " + std::to_string(cost_value));
      }
    }
    // add cost to total costs
    nlp_data_ptr_->total_cost_ += cost_value;
  }

  if (cost_debug_level_ >= 2)
    printCostValues();
}

void CostManager::addUpGradients()
{
  nlp_data_ptr_->cost_gradient_.setZero();
  for (const auto& cost_ptr : cost_ptr_vec_)
  {
    nlp_data_ptr_->cost_gradient_ += cost_ptr->getWeightedCostGradient() * nlp_data_ptr_->calculation_cache_.jacobian_map_;

    // check gradient for having a non finite value nan if option is set
    if (check_for_non_finite_values_)
    {
      // it is only neccessary to check the gradient of the cost as the jacobian of the calculation cache has allready been checked upon its calculation
      EigenUtil::checkSparseMatrixForNonFiniteValue(nlp_data_ptr_->calculation_cache_.jacobian_map_,
                                                    "Value of cost gradient named \"" + cost_ptr->getCostName() + "\" is not finite.");
    }
  }
}

}  // namespace optiminf
