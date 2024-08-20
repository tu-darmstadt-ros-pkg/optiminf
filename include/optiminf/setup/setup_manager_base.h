#pragma once



#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/manager_container.h"

namespace optiminf
{
class SetupManagerBase
{
public:
  using Ptr =std::shared_ptr<SetupManagerBase>;

  virtual ~SetupManagerBase() = default;

  /**
   * @brief setupNlp performs the steps for instantiating all classes needed to solve the optimization problem.
   * @details For instantiating custom classes use the pure virtual functions of this class. The setup procedure is executed in the following order:
   *  -# Instantiate ConstraintManager and CostManager
   *  -# call instantiatePreparationManager()
   *  -# call instantiateCalculationCache()
   *  -# call instantiateCosts()
   *  -# call instantiateConstraints()
   *  -# call instantiateSolverInterface()
   *  -# call CalculationCacheManagerBase::setupCalculationCache()
   *  -# call CostManager::setupCosts()
   *  -# call ConstraintManager::setupConstraints()
   *  -# call SolverInterfaceBase::setupSolverInterface()
   * @param manager_container_ptr
   * @param nlp_data_ptr
   */
  void setupNlp(ManagerContainer::Ptr manager_container_ptr, NlpData::Ptr nlp_data_ptr);

protected:
  /**
   * @brief instantiatePreparationManager instantiates the preparation manager and sets the pointer to it in the ManagerContainer by accessing it with getManagerContainerPtr()
   */
  virtual void instantiatePreparationManager() = 0;

  /**
   * @brief instantiateCalculationCache instantiates the calculation cache and sets the pointer to it in the ManagerContainer by accessing it with getManagerContainerPtr()
   */
  virtual void instantiateCalculationCache() = 0;

  /**
   * @brief instantiateCosts instantiates the costs and adds them to the cost manager by accessing it via the ManagerContainer using getManagerContainerPtr()
   */
  virtual void instantiateCosts() = 0;

  /**
   * @brief instantiateConstraints instantiates the constraints and adds them to the constraint manager by accessing it via the ManagerContainer using getManagerContainerPtr()

   */
  virtual void instantiateConstraints() = 0;

  /**
   * @brief instantiateSolverInterface  instantiates the solver interface and sets the pointer to it in the ManagerContainer by accessing it with getManagerContainerPtr()
   */
  virtual void instantiateSolverInterface() = 0;

  /**
   * @brief getManagerContainerPtr returns the pointer to the manager container
   * @return pointer to the manager container
   */
  const ManagerContainer::Ptr& getManagerContainerPtr() const;

  /**
   * @brief getNlpDataPtr returns the pointer to the nlp data
   * @return pointer to the nlp data
   */
  const NlpData::Ptr& getNlpDataPtr() const;

private:
  ManagerContainer::Ptr manager_container_ptr_;
  NlpData::Ptr nlp_data_ptr_;
};
}  // namespace optiminf
