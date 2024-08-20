#include "optiminf/constraints/constraint_base.h"

#include "optiminf/error_handling/optiminf_exception.h"

namespace optiminf
{
ConstraintBase::ConstraintBase(std::string constraint_name)
  : constraint_result_ptr_(std::make_shared<CalculationResult>())
  , constraint_name_(std::move(constraint_name))

{}

void ConstraintBase::setupConstraint(const NlpData::Ptr nlp_data_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr)
{
  nlp_data_ptr_ = nlp_data_ptr;
  parameter_interface_ptr_ = parameter_interface_ptr;

  setupConstraintImpl();
}

const Eigen::VectorXd& ConstraintBase::getConstraintValues() const { return constraint_result_ptr_->values_; }

const ConstSparseJacobianMap<double>& ConstraintBase::getConstraintJacobianMap() const { return constraint_result_ptr_->jacobian_map_; }

const Eigen::VectorXd& ConstraintBase::getUpperBounds() const { return upper_bounds_; }

const Eigen::VectorXd& ConstraintBase::getLowerBounds() const { return lower_bounds_; }

const std::string& ConstraintBase::getConstraintName() const { return constraint_name_; }

size_t ConstraintBase::getNrOfConstraints() const
{
  size_t nr_of_constraints = constraint_result_ptr_->values_.size();

  // throw an exception in case the number of bounds does not match the number of constraint values
  if (lower_bounds_.size() != nr_of_constraints)
  {
    throw PreparationError("The Number of lower bounds of " + constraint_name_ + " (" + std::to_string(lower_bounds_.size()) + ") is different to the number of constraints (" +
                           std::to_string(nr_of_constraints) + ")");
  }
  else if (upper_bounds_.size() != nr_of_constraints)
  {
    throw PreparationError("The Number of upper bounds of " + constraint_name_ + " (" + std::to_string(upper_bounds_.size()) + ") is different to the number of constraints(" +
                           std::to_string(nr_of_constraints) + ")");
  }

  return nr_of_constraints;
}

size_t ConstraintBase::getNumberOfConstraintViolations() const
{
  size_t nr_of_constraint_violations = 0;

  for (size_t i = 0; i < constraint_result_ptr_->values_.size(); i++)
  {
    if (lower_bounds_(i) > constraint_result_ptr_->values_(i) || constraint_result_ptr_->values_(i) > upper_bounds_(i))
    {
      nr_of_constraint_violations++;
    }
  }

  return nr_of_constraint_violations;
}

void ConstraintBase::printDebugInfo(int debug_level) const { std::cout << "No debug info implemented for constraint: " << getConstraintName() << std::endl; }

const Eigen::VectorXd& ConstraintBase::getCCVector() const { return nlp_data_ptr_->calculation_cache_.values_; }

const paraminf::ParameterInterface::Ptr& ConstraintBase::getParameterInterfacePtr() const { return parameter_interface_ptr_; }

CalculationResult& ConstraintBase::constraintValueResult() { return *constraint_result_ptr_; }

void ConstraintBase::resizeConstraintData(const size_t nr_of_constraint_values)
{
  lower_bounds_.resize(nr_of_constraint_values);
  upper_bounds_.resize(nr_of_constraint_values);
  constraint_result_ptr_->resize(nr_of_constraint_values, getCCVector().size());
}

Eigen::VectorXd& ConstraintBase::lowerBounds() { return lower_bounds_; }

Eigen::VectorXd& ConstraintBase::upperBounds() { return upper_bounds_; }

}  // namespace optiminf
