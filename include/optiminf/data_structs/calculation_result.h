#pragma once

#include <memory>

#include "optiminf/util/eigen_util.h"

namespace optiminf
{
class CalculationResult
{
public:
  using Ptr = std::shared_ptr<CalculationResult>;

  Eigen::VectorXd values_;
  ConstSparseJacobianMap<double> jacobian_map_;

  /**
   * @brief CalculationResult constructs an object representing the multidimensional result of a calculation and the jacobian with regard to the input values of the calculation
   * @details By default the jacobian map of the CalculationResult refers to an internal jacobian. In order to set it to an external jacobian call setExternalJacobianReference()
   */
  CalculationResult();

  /**
   * @brief setExternalJacobianReference sets the jacobian_map_ in order to use an external jacobian which e.g. is maintained by an auto diff tool
   * @details The member function resize() does not have an effect for the jacobian after calling this function. Do not call getInternalJacobian(),
   * applyInternalStructureChangesToMap() after calling this function.
   * @param jacobian_map Sparse matrix map which is then used for the internal jacobian map
   */
  void setExternalJacobianReference(const ConstSparseJacobianMap<double>& jacobian_map);

  /**
   * @brief getInternalJacobian this function can be used to get full access to the internal jacobian when no external jacobian is used
   * @details Use this function e.g. for changing the number or position of non zeros. After any changes that can only be done by direct access to the jacobian and not via the map
   * applyInternalStructureChangesToMap() has to be called to make sure the changes are visible when accessing through the map.
   * This function should not be called after setting an external jacobian refference by setExternalJacobianReference()
   * @return non constant reference to the internal jacobian
   */
  Eigen::SparseMatrix<double, Eigen::RowMajor>& getInternalJacobian();

  /**
   * @brief applyInternalStructureChangesToMap creates a new map that refers to the internal jacobian using the current structure of the internal jacobian
   * @details The jacobian map is only valid as long as the dimensions and the number and positions of the sparse non zeros stay the same. Once any of these are changed by using
   * the direct refference via getInternalJacobian() the changes should be applied to the map via this function before the map is used again.
   * This function should not be called after setting an external jacobian refference by setExternalJacobianReference()
   */
  void applyInternalStructureChangesToMap();

  /**
   * @brief resize resizes the values to number_of_values and if number_of_dependent_variables is non zero and the internal jacobian is used it is resized to number_of_values x
   * number_of_dependent_variables and the structural change gets applies to the map
   * @details When using an internal jacobian all non zeros are removed. When using an external jacobian the map is not affected by calling this function.
   * @param number_of_values the number of values which the object s representing
   * @param number_of_dependent_variables the number of variables that affect the values which is equal to the number of columns in the jacobian
   */
  void resize(size_t number_of_values, size_t number_of_dependent_variables = 0);

  /**
   * @brief isUsingInternalJacobian returns true as long as no external refference has been set by setExternalJacobianReference()
   * @return true if the internal jacobian is used
   */
  bool isUsingInternalJacobian();

private:
  Eigen::SparseMatrix<double, Eigen::RowMajor> internal_jacobian_;

  bool use_internal_jacobian = true;
};
}  // namespace optiminf
