#pragma once

#include "optiminf/setup/setup_manager_base.h"

namespace optiminf
{
namespace test
{
/**
 * @brief The UnitTestSetupManager class sets up an NLP for unit testing and demonstration purpose
 * @details The problem consists of three optimization variables \f$ x_0, x_1 x_2 \f$ . \n
 * The cost function is: \f$ f(x_0, x_1 x_2) = x_0 + {x_1}^2 + x_2 \f$ \n
 * The constraints are:
 * \f{eqnarray*}{
 *  x_0 &>1 \\
 *  x_1 &>2 \\
 *  x_2 &>1 \\
 *  \sqrt{{x_0}^2+x_1}+x_2 &> 10
 * \f} .
 */
class UnitTestSetupManager : public optiminf::SetupManagerBase
{
public:
  UnitTestSetupManager(bool add_cost = true, bool add_constraint = true);

  /**
   * @brief getManagerContainerPtrForTest allows to access the Manager Container in unit tests
   * @return
   */
  const ManagerContainer::Ptr& getManagerContainerPtrForTest() const;

  /**
   * @brief getNlpDataPtrForTest allows to access the nlp data
   * @return
   */
  const NlpData::Ptr& getNlpDataPtrForTest() const;

private:
  void instantiatePreparationManager() final;

  void instantiateCalculationCache() final;

  void instantiateCosts() final;

  void instantiateConstraints() final;

  void instantiateSolverInterface() final;

  bool add_cost_;
  bool add_constraint_;
};
}  // namespace test
}  // namespace optiminf
