#pragma once

#include "paraminf/parameter_interface.h"

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/calculation_result.h"

namespace optiminf
{
class ConstraintBase
{
public:
  using Ptr = std::shared_ptr<ConstraintBase>;

  /**
   * @brief ConstraintBase base constructor for the constraint class that should be called by the child class to set the name and the reference to the nlp data struct
   * @param constraint_name the name of the constraint that can be used for diagnostic or debug output
   * @param nlp_data_ptr reference to the nlp data struct
   */
  ConstraintBase(std::string constraint_name);

  virtual ~ConstraintBase() = default;

  virtual void setupConstraint(const NlpData::Ptr nlp_data_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr);

  /**
   * @brief prepareConstraint is called everytime before the solver is started.
   * @details If the boundaries of the constraint are changing for each problem to be solved, than they should be set during this call.  If there are calculations that can be
   * precomputed for every iteration but not for consecutive calls to solver these should also be done during this call.
   */
  virtual void prepareConstraint() = 0;

  /**
   * @brief updateValues update the constraint values accessible through constraintValues() according to the calculation cache values that can be read using getCCVector()
   */
  virtual void updateValues() = 0;

  /**
   * @brief updateJacobian update the constraint jacobian referenced by the jacobian map according to the calculation cache values that can be read using getCCVector()
   */
  virtual void updateJacobian() = 0;

  /**
   * @brief getConstraintValues return read access to the vector containing the constraint values
   * @return read access to the vector containing the constraint values
   */
  const Eigen::VectorXd& getConstraintValues() const;

  /**
   * @brief getConstraintJacobianMap return read access to the jacobian of the constraint
   * @return read access to the jacobian of the constraint
   */
  const ConstSparseJacobianMap<double>& getConstraintJacobianMap() const;

  /**
   * @brief getLowerBounds return read access to the vector containing the values of the lower bounds
   * @return read access to the vector containing the values of the lower bounds
   */
  const Eigen::VectorXd& getLowerBounds() const;

  /**
   * @brief getUpperBounds return read access to the vector containing the values of the upper bounds
   * @return read access to the vector containing the values of the upper bounds
   */
  const Eigen::VectorXd& getUpperBounds() const;

  /**
   * @brief getConstraintName returns the constraint name in order to use it for diagnostic or debug purposes
   * @return the constraint name
   */
  const std::string& getConstraintName() const;

  /**
   * @brief getNrOfConstraints
   * @return number of currently active values that are constraint
   */
  size_t getNrOfConstraints() const;

  /**
   * @brief getNumberOfConstraintViolations returns the number of constraint values that violate its boundary
   * @return number of constraint values that violate its boundary
   */
  size_t getNumberOfConstraintViolations() const;

  /**
   * @brief printDebugInfo is intended to be overwritten by each child class alowing to get constraint specific debug information
   * @details An example for further debug information is e.g. for which values a constraint is violated an can or could even be a more detailed information like at which point in
   * time a spline violates a boundary
   */
  virtual void printDebugInfo(int debug_level) const;

protected:
  /**
   * @brief getCCVector read accessor to the calculation cache vector intended to be used for updating the constraint values
   * @return read accessor to the calculation cache
   */
  const Eigen::VectorXd& getCCVector() const;

  /** @brief Get a shared pointer to the parmeter interface.
   * @return pointer to the parmeter interface
   */
  const paraminf::ParameterInterface::Ptr& getParameterInterfacePtr() const;

  /**
   * @brief constraintValueResult read and write accessor intended for updating the constraint values and constraint jacobian
   * @return read and write accessor to constraint value result
   */
  CalculationResult& constraintValueResult();

  /**
   * @brief lowerBounds read and write accessor intended for the setting the lower bounds
   * @return read and write accessor to lower bounds
   */
  Eigen::VectorXd& lowerBounds();

  /**
   * @brief upperBounds read and write accessor intended for the setting the upper bounds
   * @return read and write accessor to upper bounds
   */
  Eigen::VectorXd& upperBounds();

  /**
   * @brief resizeConstraintData resizes the bounds and constraints to the number of constraints. If the internal jacobian of the constraint value result is used it gets resized to
   * nr_of_constraint_values x size of cc vector
   * @param nr_of_constraint_values
   */
  void resizeConstraintData(size_t nr_of_constraint_values);

private:
  const std::string constraint_name_;
  NlpData::Ptr nlp_data_ptr_;
  paraminf::ParameterInterface::Ptr parameter_interface_ptr_;

  const CalculationResult::Ptr constraint_result_ptr_;

  Eigen::VectorXd lower_bounds_;
  Eigen::VectorXd upper_bounds_;

  /**
   * @brief setupConstraintImpl is called by setupConstraint()
   */
  virtual void setupConstraintImpl() = 0;
};
}  // namespace optiminf
