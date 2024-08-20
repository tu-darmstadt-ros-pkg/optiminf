#pragma once

#include "paraminf/parameter_interface.h"

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/costs/cost_base.h"

namespace optiminf
{
class CostManager
{
public:
  using Ptr = std::shared_ptr<CostManager>;

  CostManager(NlpData::Ptr nlp_data_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr);

  /**
   * @brief setupCosts calls setup for all costs that have been added
   *  */
  void setupCosts();

  /**
   * @brief prepare calls prepare for all costs that have been added
   */
  void prepareCosts();

  /**
   * @brief updateCostValue updates the value of all costs and sets the cost value in nlp data to its sum
   */
  void updateCostValue();

  /**
   * @brief updateCostGradient updates the value of all costs gradients and sets the cost gradient value in nlp data to its sum
   */
  void updateCostGradient();

  /**
   * @brief addCost adds a shared pointer to a child class of CostBase to consider this cost for setting up and solving the nlp
   * @param constraint_ptr shared pointer to a child class of CostBase
   */
  void addCost(CostBase::Ptr cost_ptr);

  /**
   * @brief printCostValues prints out the weighted value for each cost object
   */
  void printCostValues() const;

private:
  void addUpCosts();
  void addUpGradients();

  const NlpData::Ptr nlp_data_ptr_;
  const paraminf::ParameterInterface::Ptr parameter_interface_ptr_;

  std::vector<CostBase::Ptr> cost_ptr_vec_;

  int cost_debug_level_ = 0;
  bool check_for_non_finite_values_ = false;
};
}  // namespace optiminf
