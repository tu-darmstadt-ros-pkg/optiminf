#include "optiminf/constraints/constraint_manager.h"

#include "optiminf/util/parameter_naming_util.h"

#include "optiminf/util/eigen_util.h"

namespace optiminf
{
ConstraintManager::ConstraintManager(const NlpData::Ptr nlp_data_ptr, const paraminf::ParameterInterface::Ptr parameter_interface_ptr)
  : nlp_data_ptr_(nlp_data_ptr)
  , parameter_interface_ptr_(parameter_interface_ptr)
{}

void ConstraintManager::setupConstraints()
{
  parameter_interface_ptr_->getParam(ParameterNamingUtil::CONSTRAINT_DEBUG_LEVEL_PARAM_NAME, constraint_debug_level_);
  parameter_interface_ptr_->getParam(ParameterNamingUtil::CHECK_FOR_NON_FINITE_VALUES_PARAM_NAME, check_for_non_finite_values_);

  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    constraint_ptr->setupConstraint(nlp_data_ptr_, parameter_interface_ptr_);
  }
}

void ConstraintManager::prepareConstraints()
{
  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    constraint_ptr->prepareConstraint();
  }

  size_t nr_of_constraint_values = 0;

  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    nr_of_constraint_values += constraint_ptr->getLowerBounds().size();
  }

  // resize the data structures holding the constraint data
  nlp_data_ptr_->lower_constraint_bounds_.resize(nr_of_constraint_values);
  nlp_data_ptr_->upper_constraint_bounds_.resize(nr_of_constraint_values);
  nlp_data_ptr_->constraint_values_.resize(nr_of_constraint_values);

  combineConstraintBoundaries();

  size_t nr_of_opt_vars = nlp_data_ptr_->optimization_variables_.size();

  nlp_data_ptr_->constraint_jacobian_.resize(nr_of_constraint_values, nr_of_opt_vars);
}

void ConstraintManager::updateConstraintValues()
{
  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    constraint_ptr->updateValues();
  }
  combineConstraintValues();
}

void ConstraintManager::updateConstraintJacobian()
{
  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    constraint_ptr->updateJacobian();
  }
  combineConstraintJacobian();
}

void ConstraintManager::addConstraint(const ConstraintBase::Ptr constraint_ptr) { constraint_ptr_vec_.push_back(constraint_ptr); }

double ConstraintManager::getMaximumConstraintViolation() const
{
  double maximal_violation = 0.0;
  size_t nr_of_constraints = nlp_data_ptr_->constraint_values_.size();
  for (size_t i = 0; i < nr_of_constraints; i++)
  {
    double upper_bound_violation = nlp_data_ptr_->constraint_values_(i) - nlp_data_ptr_->upper_constraint_bounds_(i);
    double lower_bound_violation = nlp_data_ptr_->lower_constraint_bounds_(i) - nlp_data_ptr_->constraint_values_(i);

    maximal_violation = std::max(maximal_violation, upper_bound_violation);
    maximal_violation = std::max(maximal_violation, lower_bound_violation);
  }
  return maximal_violation;
}

void ConstraintManager::printNrOfConstraintViolations() const
{
  std::cout << "Number of constraint violations:" << std::endl;
  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    std::cout << "\t" << constraint_ptr->getConstraintName() << ": " << constraint_ptr->getNumberOfConstraintViolations() << std::endl;
  }
}

void ConstraintManager::printDebugInfo() const
{
  std::cout << "Constraint Debug Info:" << std::endl;
  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    constraint_ptr->printDebugInfo(constraint_debug_level_);
  }
}

void ConstraintManager::combineConstraintBoundaries()
{
  size_t start_index = 0;

  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    size_t constraints_to_add = constraint_ptr->getNrOfConstraints();

    nlp_data_ptr_->lower_constraint_bounds_.segment(start_index, constraints_to_add) = constraint_ptr->getLowerBounds();
    nlp_data_ptr_->upper_constraint_bounds_.segment(start_index, constraints_to_add) = constraint_ptr->getUpperBounds();

    // check values to be finite if option is set
    if (check_for_non_finite_values_)
    {
      EigenUtil::checkVectorForNonFiniteValue(nlp_data_ptr_->lower_constraint_bounds_.segment(start_index, constraints_to_add),
                                              "Value of lower boundary of constraint named \"" + constraint_ptr->getConstraintName() + "\" is not finite.");
      EigenUtil::checkVectorForNonFiniteValue(nlp_data_ptr_->upper_constraint_bounds_.segment(start_index, constraints_to_add),
                                              "Value of upper boundary of constraint named \"" + constraint_ptr->getConstraintName() + "\" is not finite.");
    }

    start_index += constraints_to_add;
  }
}

void ConstraintManager::combineConstraintValues()
{
  size_t start_index = 0;

  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    size_t constraints_to_add = constraint_ptr->getNrOfConstraints();

    nlp_data_ptr_->constraint_values_.segment(start_index, constraints_to_add) = constraint_ptr->getConstraintValues();

    // check values to be finite if option is set
    if (check_for_non_finite_values_)
    {
      EigenUtil::checkVectorForNonFiniteValue(nlp_data_ptr_->constraint_values_.segment(start_index, constraints_to_add),
                                              "Value of constraint named \"" + constraint_ptr->getConstraintName() + "\" is not finite.");
    }

    start_index += constraints_to_add;
  }

  if (constraint_debug_level_ >= 3)
    printNrOfConstraintViolations();
  if (constraint_debug_level_ >= 4)
    printDebugInfo();
}

void ConstraintManager::combineConstraintJacobian()
{
  size_t start_index = 0;

  for (const auto& constraint_ptr : constraint_ptr_vec_)
  {
    size_t constraints_to_add = constraint_ptr->getNrOfConstraints();
    for (size_t i = 0; i < constraints_to_add; i++)
    {
      nlp_data_ptr_->constraint_jacobian_.row(start_index + i) = constraint_ptr->getConstraintJacobianMap().row(i) * nlp_data_ptr_->calculation_cache_.jacobian_map_;
    }

    start_index += constraints_to_add;

    // check values to be finite if option is set
    if (check_for_non_finite_values_)
    {
      // it is only neccessary to check the jacobian of the constraint as the jacobian of the calculation cache has allready been checked upon its calculation
      EigenUtil::checkSparseMatrixForNonFiniteValue(constraint_ptr->getConstraintJacobianMap(),
                                                    "Jacobian of Constraint named \"" + constraint_ptr->getConstraintName() + "\" contains non finite value. ");
    }
  }
}

}  // namespace optiminf
