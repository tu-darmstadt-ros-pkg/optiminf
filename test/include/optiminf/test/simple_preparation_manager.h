#pragma once

#include "optiminf/preparation/preparation_manager_base.h"

namespace optiminf
{
namespace test
{
/**
 * @brief The SimplePreparationManager class can be used for preparing the example NLP
 * @details It initializes the optimization varibales with the following values:
 * \f{eqnarray*}{
 *  x_0 &=& 1 \\
 *  x_1 &=& 2 \\
 *  x_2 &=& 5 \\
 * \f}
 */
class SimplePreparationManager : public optiminf::PreparationManagerBase
{
public:
  SimplePreparationManager(std::shared_ptr<optiminf::ManagerContainer> manager_container_ptr, optiminf::NlpData::Ptr nlp_data_ptr);

protected:
  void initializeOptimizationVariables() final;
};
}  // namespace test
}  // namespace optiminf
