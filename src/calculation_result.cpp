#include "optiminf/data_structs/calculation_result.h"

namespace optiminf
{
CalculationResult::CalculationResult()
  : jacobian_map_(0, 0, 0, nullptr, nullptr, nullptr)
{}

void CalculationResult::setExternalJacobianReference(const ConstSparseJacobianMap<double>& jacobian_map)
{
  // this is not a memory allocation but rather replaces the map with a new one (see https://eigen.tuxfamily.org/dox/group__TutorialMapClass.html)
  new (&jacobian_map_) ConstSparseJacobianMap<double>(jacobian_map);
  use_internal_jacobian = false;
}

Eigen::SparseMatrix<double, Eigen::RowMajor>& CalculationResult::getInternalJacobian()
{
  assert(use_internal_jacobian == true);
  return internal_jacobian_;
}

void CalculationResult::applyInternalStructureChangesToMap()
{
  assert(use_internal_jacobian == true);
  new (&jacobian_map_) ConstSparseJacobianMap<double>(EigenUtil::getMapFromMatrix(internal_jacobian_));
}

void CalculationResult::resize(size_t number_of_values, size_t number_of_dependent_variables)
{
  values_.resize(number_of_values, 1);
  if (use_internal_jacobian && number_of_dependent_variables != 0)
  {
    internal_jacobian_.resize(number_of_values, number_of_dependent_variables);
    internal_jacobian_.setZero();
    applyInternalStructureChangesToMap();
  }
}

bool CalculationResult::isUsingInternalJacobian() { return use_internal_jacobian; }

}  // namespace optiminf
