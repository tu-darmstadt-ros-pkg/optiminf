#pragma once

#include "paraminf/parameter_interface.h"

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/calculation_result.h"

namespace optiminf
{
class CostBase
{
public:
  using Ptr = std::shared_ptr<CostBase>;

  /**
   * @brief CostBase base constructor for the cosr class that should be called by the child class to set the name and the reference to the nlp data struct
   * @param cost_name the name of the cost that can be used for diagnostic or debug output
   * @param nlp_data_ptr reference to the nlp data struct
   */
  CostBase(std::string cost_name, double weight);
  virtual ~CostBase() = default;

  /**
   * @brief setupCost is called after all objects are instatiated during the setup phase and invokes setupCostImpl()
   */
  virtual void setupCost(const NlpData::Ptr nlp_data_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr);

  /**
   * @brief This function is called everytime before the solver is started.
   * @details If there are calculations that can be precomputed for every iteration but not for consecutive calls to solver these should be done during this call
   */
  virtual void prepareCost() = 0;

  /**
   * @brief updateValue update the cost value accessible through unweightedCostValue() according to the calculation cache values that can be read using getCCVector()
   */
  virtual void updateValue() = 0;

  /**
   * @brief updateGradient update the cost gradient referenced by the jacobian map according to the calculation cache values that can be read using getCCVector()
   */
  virtual void updateGradient() = 0;

  const std::string& getCostName() const;

  /**
   * @brief getWeightedCostValue returns the current weighted cost value
   * @return weighted cost
   */
  double getWeightedCostValue() const;

  /**
   * @brief getWeightedCostGradient returns the gradient for the weighted cost value
   * @return  weighted gradient
   */
  Eigen::SparseMatrix<double, Eigen::RowMajor> getWeightedCostGradient() const;

protected:
  /**
   * @brief getCCVector read accessor to the calculation cache vector intended to be used for updating the constraint values
   * @return read accessor to the calculation cache
   */
  const Eigen::VectorXd& getCcVector() const;

  /** @brief Get a shared pointer to the parmeter interface.
   * @return pointer to the parmeter interface
   */
  const paraminf::ParameterInterface::Ptr& getParameterInterfacePtr() const;

  /**
   * @brief unweightedCostResult should be used to access the unweighted cost values and jacobian
   * @return reference to cost result
   */
  CalculationResult& unweightedCostResult();

private:
  const std::string cost_name_;
  const double weight_;

  NlpData::Ptr nlp_data_ptr_;
  paraminf::ParameterInterface::Ptr parameter_interface_ptr_;

  const CalculationResult::Ptr cost_result_ptr_;

  /**
   * @brief setupCostImpl is called by setupCost()
   */
  virtual void setupCostImpl() = 0;
};
}  // namespace optiminf
