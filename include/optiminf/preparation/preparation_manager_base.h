#pragma once

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/manager_container.h"

namespace optiminf
{
class ManagerContainer;

class PreparationManagerBase
{
public:
  using Ptr = std::shared_ptr<PreparationManagerBase>;

  PreparationManagerBase(std::shared_ptr<ManagerContainer> manager_container_ptr, NlpData::Ptr nlp_data_ptr);

  virtual ~PreparationManagerBase() = default;

  /**
   * @brief prepareNlp performs all necessary steps that have to be done before the solver can be started.
   * @details Unlike the setup procedure this is called every time before solving the problem. The steps are executed in the following order:
   *  -# initialize optimization variables
   *  -# prepare calculation cache
   *  -# prepare costs
   *  -# prepare constraints
   */
  virtual void prepareNlp();

protected:
  /**
   * @brief initializeOptimizationVariables should be implemented by the child class to resize the optimization variable vector in the nlp data struct and provide an initial guess
   * for the solution.
   */
  virtual void initializeOptimizationVariables() = 0;

  const std::shared_ptr<ManagerContainer>& getManagerContainerPtr() const;

  const NlpData::Ptr& getNlpDataPtr() const;

private:
  const std::shared_ptr<ManagerContainer> manager_container_ptr_;
  const NlpData::Ptr nlp_data_ptr_;
};
}  // namespace optiminf
