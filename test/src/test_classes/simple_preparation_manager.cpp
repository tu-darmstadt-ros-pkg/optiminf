#include "optiminf/test/simple_preparation_manager.h"
namespace optiminf
{
namespace test
{
SimplePreparationManager::SimplePreparationManager(const std::shared_ptr<optiminf::ManagerContainer> manager_container_ptr, const optiminf::NlpData::Ptr nlp_data_ptr)
  : optiminf::PreparationManagerBase(manager_container_ptr, nlp_data_ptr)
{}

void SimplePreparationManager::initializeOptimizationVariables()
{
  // resize the vector containing the optimization variables
  getNlpDataPtr()->optimization_variables_.resize(3, 1);

  // provide an initial guess for the values of the optimization variables
  getNlpDataPtr()->optimization_variables_(0) = 1;
  getNlpDataPtr()->optimization_variables_(1) = 2;
  getNlpDataPtr()->optimization_variables_(2) = 5;
}
}  // namespace test
}  // namespace optiminf
