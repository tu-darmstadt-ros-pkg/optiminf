#include "optiminf/costs/cost_base.h"

namespace optiminf
{
CostBase::CostBase(std::string cost_name, double weight)
  : cost_result_ptr_(std::make_shared<CalculationResult>())
  , cost_name_(std::move(cost_name))
  , weight_(weight)
{
  cost_result_ptr_->values_.resize(1);
}

void CostBase::setupCost(const NlpData::Ptr nlp_data_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr)
{
  nlp_data_ptr_ = nlp_data_ptr;
  parameter_interface_ptr_ = parameter_interface_ptr;

  setupCostImpl();
}

const std::string& CostBase::getCostName() const { return cost_name_; }

double CostBase::getWeightedCostValue() const { return cost_result_ptr_->values_(0) * weight_; }

Eigen::SparseMatrix<double, Eigen::RowMajor> CostBase::getWeightedCostGradient() const { return cost_result_ptr_->jacobian_map_ * weight_; }

const Eigen::VectorXd& CostBase::getCcVector() const { return nlp_data_ptr_->calculation_cache_.values_; }

CalculationResult& CostBase::unweightedCostResult() { return *cost_result_ptr_; }

const paraminf::ParameterInterface::Ptr& CostBase::getParameterInterfacePtr() const { return parameter_interface_ptr_; }

}  // namespace optiminf
