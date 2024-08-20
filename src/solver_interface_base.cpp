#include "optiminf/solver_interface/solver_interface_base.h"

#include "optiminf/util/parameter_naming_util.h"
namespace optiminf
{
void SolverInterfaceBase::setupSolverInterface(const std::shared_ptr<ManagerContainer> manager_container_ptr, const NlpData::Ptr nlp_data_ptr)
{
  manager_container_ptr_ = manager_container_ptr;
  nlp_data_ptr_ = nlp_data_ptr;

  setupSolver();

  /* parse parameter */
  getParameterInterfacePtr()->getParam(ParameterNamingUtil::ACCEPTABLE_CONSTRAINT_VIOLATION_ON_SOLVER_FAILURE_PARAM_NAME, acceptable_constraint_violation_);
  std::vector<int> acceptable_failures;
  getParameterInterfacePtr()->getParam(ParameterNamingUtil::ACCEPTABLE_SOLVER_FAILURES_PARAM_NAME, acceptable_failures);
  acceptable_solver_failures_.resize(acceptable_failures.size());
  for (size_t i = 0; i < acceptable_failures.size(); i++)
  {
    acceptable_solver_failures_[i] = static_cast<SolverResultStatus>(acceptable_failures[i]);
  }
}

SolverResultStatus SolverInterfaceBase::runSolver()
{
  SolverResultStatus return_status = runSolverImpl();

  if (static_cast<int>(return_status) < 0 && manager_container_ptr_->constraint_manager_ptr_->getMaximumConstraintViolation() <= acceptable_constraint_violation_)
  {
    if (return_status == SolverResultStatus::IterationLimitReached)
    {
      return_status = SolverResultStatus::FeasibleSolutionIterationLimitReached;
    }
    else if (return_status == SolverResultStatus::TimeLimitReached)
    {
      return_status = SolverResultStatus::FeasibleSoultionTimeLimitReached;
    }
    else if (acceptable_solver_failures_.size() > 0 &&
             (static_cast<int>(acceptable_solver_failures_[0]) == 0 ||  // this is used as a wield card for ignoring any issues in case the maximum constraint violation is below
                                                                        // acceptable_constraint_violation_
              std::find(acceptable_solver_failures_.begin(), acceptable_solver_failures_.end(), return_status) != acceptable_solver_failures_.end()))
    {
      return_status = SolverResultStatus::FeasibleSolutionDespiteError;
    }
  }
  return return_status;
}

const NlpData::Ptr& SolverInterfaceBase::getNlpDataPtr() const { return nlp_data_ptr_; }

const paraminf::ParameterInterface::Ptr& SolverInterfaceBase::getParameterInterfacePtr() const { return manager_container_ptr_->parameter_interface_ptr_; }

void SolverInterfaceBase::updateCostValue(const bool new_input_values)
{
  manager_container_ptr_->calculation_cache_manager_ptr_->updateCacheStatus(new_input_values);
  manager_container_ptr_->calculation_cache_manager_ptr_->ensureValidCacheValues();
  manager_container_ptr_->cost_manager_ptr_->updateCostValue();
}

void SolverInterfaceBase::updateConstraintValues(const bool new_input_values)
{
  manager_container_ptr_->calculation_cache_manager_ptr_->updateCacheStatus(new_input_values);
  manager_container_ptr_->calculation_cache_manager_ptr_->ensureValidCacheValues();
  manager_container_ptr_->constraint_manager_ptr_->updateConstraintValues();
}

void SolverInterfaceBase::updateCostGradient(const bool new_input_values)
{
  manager_container_ptr_->calculation_cache_manager_ptr_->updateCacheStatus(new_input_values);
  manager_container_ptr_->calculation_cache_manager_ptr_->ensureValidCacheValues();
  manager_container_ptr_->calculation_cache_manager_ptr_->ensureValidCacheJacobian();
  manager_container_ptr_->cost_manager_ptr_->updateCostGradient();
}

void SolverInterfaceBase::updateConstraintJacobian(const bool new_input_values)
{
  manager_container_ptr_->calculation_cache_manager_ptr_->updateCacheStatus(new_input_values);
  manager_container_ptr_->calculation_cache_manager_ptr_->ensureValidCacheValues();
  manager_container_ptr_->calculation_cache_manager_ptr_->ensureValidCacheJacobian();
  manager_container_ptr_->constraint_manager_ptr_->updateConstraintJacobian();
}

}  // namespace optiminf
