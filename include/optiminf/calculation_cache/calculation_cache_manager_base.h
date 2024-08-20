#pragma once

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/manager_container.h"

namespace optiminf
{
class ManagerContainer;

class CalculationCacheManagerBase
{
public:
  using Ptr = std::shared_ptr<CalculationCacheManagerBase>;
  using ConstPtr = std::shared_ptr<const CalculationCacheManagerBase>;

  CalculationCacheManagerBase(std::shared_ptr<const ManagerContainer> manager_container_ptr, NlpData::Ptr nlp_data_ptr);

  virtual ~CalculationCacheManagerBase() = default;

  /**
   * @brief setupCalculationCache is called once all during the call of NlpManager::setupNlp() to instantiate all necessary components, like e.g. splines or auto diff
   * tools
   * @details If the size of the calculation cache is independent of the preparation phase, that it can also be resized in this step. Otherwise use prepareCalculationCache for
   * resizing.
   */
  virtual void setupCalculationCache() = 0;

  /**
   * @brief prepareCalculationCache prepares the calculation cache for the current problem configuration before the solving procedure is started
   */
  virtual void prepareCalculationCache() = 0;

  /**
   * @brief updateCacheStatus updates the cache status before calls of ensureValidCacheValues() or ensureValidCacheJacobian()
   * @details This method has to be called with "true" when the parameter vector or optimization variable vector have been updated since the last call ensureValidCacheValues() or
   * ensureValidCacheJacobian() in order to ensure the new values are considered when calling ensureValidCacheValues() or ensureValidCacheJacobian() again. Calls with false have no
   * effect.
   * @param new_input_values true if values have been updated
   */
  void updateCacheStatus(bool new_input_values);

  /**
   * @brief ensureValidCacheValues calculates the cache values if neccessary
   * @details The calculation is performed if the cache has not yet calculated the values at all or the cache is invalidated after a call to this function. The values used for the
   * calculation are the values at the moment of this function call
   */
  void ensureValidCacheValues();

  /**
   * @brief ensureValidCacheJacobian calculates the cache jacobian if neccessary
   * @details The calculation is performed if the cache has not yet calculated the values at all or the cache is invalidated after a call to this function. The values used for the
   * calculation are the values at the moment of this function call
   */
  void ensureValidCacheJacobian();

protected:
  /**
   * @brief getOptVarVec
   * @return const reference to the vector holding the optimization variables
   */
  const Eigen::VectorXd& getOptVarVec() const;

  /**
   * @brief getParamVec
   * @return const reference to the vector holding the parameters
   */
  const Eigen::VectorXd& getParamVec() const;

  /**
   * @brief updateValues update the values of the calulation cache
   * @details For implementing this function in the derived class use getOptVarVec(), getParamVec() and ccVector() for accessing the required data
   */
  virtual void updateValues() = 0;

  /**
   * @brief updateJacobian update the jacobian of the calulation cache
   * @details For implementing this function in the derived class use getOptVarVec(), getParamVec() for accessing the required data and manipulate the map that has been set by
   * setJacobianMap().
   */
  virtual void updateJacobian() = 0;

  /**
   * @brief calculationCacheResult
   * @return reference to the calculation cache result
   */
  CalculationResult& calculationCacheResult();

  /**
   * @brief getManagerContainerPtr returns a reference to the manager container
   * @return reference to the manager container
   */
  std::shared_ptr<const ManagerContainer> getManagerContainerPtr();

private:
  const NlpData::Ptr nlp_data_ptr_;
  const std::shared_ptr<const ManagerContainer> manager_container_ptr_;

  bool calculation_cache_values_valid_ = false;
  bool calculation_cache_jacobian_valid_ = false;

  bool check_for_non_finite_values_ = false;
};

}  // namespace optiminf
