#pragma once
#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/manager_container.h"

#include "optiminf/solver_interface/solver_result_status.h"
namespace optiminf
{
class ManagerContainer;

class SolverInterfaceBase
{
public:
  using Ptr = std::shared_ptr<SolverInterfaceBase>;

  virtual ~SolverInterfaceBase() = default;

  /**
   * @brief This method is used to setup the solver interface during the initialization phase at the start of the program.
   * @details While this retrievs the neccessary pointer also setupSolver() is called
   * @param manager_container_ptr
   * @param nlp_data_ptr
   * @param parameter_interface_ptr
   */
  void setupSolverInterface(std::shared_ptr<ManagerContainer> manager_container_ptr, NlpData::Ptr nlp_data_ptr);

  /**
   * @brief prepareSolver configures the solver for the current problem dependent on the number of constraints, optimization variables and the sparsity pattern of the cost
   * gradient and constrain jacobian.
   * @details This function should perform all memory allocations that can not be done during the setup phase when setupSolver() is called.
   */
  virtual void prepareSolver() = 0;

  /**
   * @brief runSolver starts the solver and blocks until the solver has finished or was aborted.
   * @return the SolverResultStatus after the solver has stoped.
   */
  SolverResultStatus runSolver();

  /**
   * @brief stopSolver stops the solver in a way that the optimization variables in nlp data are valid and represent the values of the last successful iteration.
   */
  virtual void stopSolver() = 0;

protected:
  /**
   * @brief runSolverImpl fuction that should be implemented and is responsible for the actual solving of the problem.
   * @return the SolverResultStatus after the solver has stoped
   */
  virtual SolverResultStatus runSolverImpl() = 0;

  /**
   * @brief Get a shared pointer to the NlpData object
   * @return pointer to the NlpData object
   */
  const NlpData::Ptr& getNlpDataPtr() const;

  /**
   * @brief Get a shared pointer to the parmeter interface.
   * @return pointer to the parmeter interface
   */
  const paraminf::ParameterInterface::Ptr& getParameterInterfacePtr() const;

  /**
   * @brief setupSolver does all memory allocations that are independent of the the number of constraints, optimization variables and the sparsity
   * pattern of the cost gradient and constrain jacobian.
   */
  virtual void setupSolver() = 0;

  /**
   * @brief updateCostValue this function should be called by the solver interface implementation to calculate the cost value in nlp_data depending on the current optimization
   * variables.
   * @param new_input_values should be set true if the optimization variable have been updated since the last call of updateCostValue(), updateConstraintValues(),
   * updateCostGradient() or updateConstraintJacobian()
   */
  void updateCostValue(bool new_input_values);

  /**
   * @brief updateConstraintValues this function should be called by the solver interface implementation to update the constraint values in nlp_data depending on
   * the current optimization variables.
   */
  void updateConstraintValues(bool new_input_values);

  /**
   * @brief updateCostGradient this function should be called by the solver interface implementation to update the cost gradient in nlp_data depending on
   * the current optimization variables
   */
  void updateCostGradient(bool new_input_values);

  /**
   * @brief updateConstraintJacobian this function should be called by the solver interface implementation to update the constraint jacobian in nlp_data depending on
   * the current optimization variables.
   */
  void updateConstraintJacobian(bool new_input_values);

  /**
   * @brief iterationDoneCallback  this function should be called by the solver interface implementation after the new values for the optimization variables are calculated to
   * indicate that an iteration is finished.
   * @details This can be used to trigger functionality for logging, performance metrics and debugging.
   */
  void iterationDoneCallback();

private:
  /**
   * @brief nlp_data_ptr_ contains the optimization variables to be updated by the solver and the data structures representing the results of updateCostValue(),
   * updateConstraintValues(), updateCostGradient(), updateConstraintJacobian()
   */
  NlpData::Ptr nlp_data_ptr_;

  std::shared_ptr<ManagerContainer> manager_container_ptr_;

  double acceptable_constraint_violation_ = -1.0;
  std::vector<SolverResultStatus> acceptable_solver_failures_;
};
}  // namespace optiminf
