#include "optiminf/test/simple_calculation_cache.h"

#include "autojac/function_generation_util.h"

#include "optiminf/util/eigen_util.h"
#include "optiminf/util/parameter_naming_util.h"

namespace optiminf
{
namespace test
{
SimpleCalculationCache::SimpleCalculationCache(optiminf::ManagerContainer::ConstPtr manager_container_ptr, NlpData::ConstPtr nlp_data_ptr)
  : optiminf::CalculationCacheManagerBase(manager_container_ptr, nlp_data_ptr)
{}

void SimpleCalculationCache::setupCalculationCache()
{
  using CGD = CppAD::cg::CG<double>;
  using ADCG = CppAD::AD<CGD>;

  const size_t nr_of_cache_values = 7;

  const size_t nr_of_opt_variables = 3;

  // allocate space for cache variables
  calculationCacheResult().resize(nr_of_cache_values, nr_of_opt_variables);

  // create autodiff jacobian
  CppAD::ADFun<CGD> func = autojac::generateCppAdFunction<CGD>(nr_of_opt_variables, nr_of_cache_values, &calculateCacheValues<ADCG>);

  const std::string file_path = getManagerContainerPtr()->parameter_interface_ptr_->getParam<std::string>("problem/code_generation_file_path");
  FileSystemUtil::createDirectoryIfNonexistent(file_path);
  cppad_jacobian_ptr_ = std::make_shared<autojac::SparseJacobianCodeGen<double>>(nr_of_opt_variables, nr_of_cache_values, func, file_path + "/cache_jacobian_library");

  // set jacobian map
  calculationCacheResult().setExternalJacobianReference(cppad_jacobian_ptr_->getMap());
}

void SimpleCalculationCache::prepareCalculationCache() {}

void SimpleCalculationCache::updateValues() { calculateCacheValues(getOptVarVec(), calculationCacheResult().values_); }

void SimpleCalculationCache::updateJacobian() { cppad_jacobian_ptr_->updateJacobian(getOptVarVec()); }
}  // namespace test
}  // namespace optiminf
