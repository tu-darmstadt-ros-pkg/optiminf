#pragma once

#include "paraminf/parameter_interface.h"

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/constraints/constraint_base.h"

namespace optiminf
{
class ConstraintManager
{
public:
  using Ptr = std::shared_ptr<ConstraintManager>;

  ConstraintManager(NlpData::Ptr nlp_data_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr);
  /**
   * @brief setupConstraints calls setup for all constraints that have been added
   */
  void setupConstraints();

  /**
   * @brief prepareConstraints calls prepare for all constraints that have been added and combines the boundaries of the constraints and adds the to nlp data
   */
  void prepareConstraints();

  /**
   * @brief updateConstraintValues updates constraint values in nlp data
   */
  void updateConstraintValues();

  /**
   * @brief updateConstraintJacobians updates the constraint jacobian in nlp data
   */
  void updateConstraintJacobian();

  /**
   * @brief addConstraint add a shared pointer to a child class of ConstraintBase to consider this constraint for setting up and solving the nlp
   * @param constraint_ptr shared pointer to a child class of ConstraintBase
   */
  void addConstraint(ConstraintBase::Ptr constraint_ptr);

  /**
   * @brief getMaximumConstraintViolation determines the maximal violation of all constraints.
   * @details The value is always positiv independent whether a upper or lower bound is violated
   * @return maximal violation of all constraints or zero if all consraints are satisfied
   */
  double getMaximumConstraintViolation() const;

  /**
   * @brief printNrOfConstraintViolations prints for each constraint the number of values that violate a boundary
   */
  void printNrOfConstraintViolations() const;

  /**
   * @brief printDebugInfo calls the printDebugInfo() method of all constraints
   */
  void printDebugInfo() const;

private:
  void combineConstraintBoundaries();
  void combineConstraintValues();
  void combineConstraintJacobian();

  const NlpData::Ptr nlp_data_ptr_;
  const paraminf::ParameterInterface::Ptr parameter_interface_ptr_;

  std::vector<ConstraintBase::Ptr> constraint_ptr_vec_;

  int constraint_debug_level_ = 0;
  bool check_for_non_finite_values_ = false;
};
}  // namespace optiminf
