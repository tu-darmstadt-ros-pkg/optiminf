#include "optiminf/test/simple_cost.h"

#include "autojac/function_generation_util.h"

#include "optiminf/test/simple_cc_view.h"
#include "optiminf/util/eigen_util.h"

namespace optiminf
{
namespace test
{
SimpleCost::SimpleCost()
  : CostBase("ExampleCost", 2)
{}

void SimpleCost::setupCostImpl()
{
  using CGD = CppAD::cg::CG<double>;
  using ADCG = CppAD::AD<CGD>;

  const size_t nr_of_cache_values = getCcVector().size();

  // create autodiff jacobian

  CppAD::ADFun<CGD> func = autojac::generateCppAdFunction<CGD>(nr_of_cache_values, 1, &calculateCostValue<ADCG>);

  const std::string file_path = getParameterInterfacePtr()->getParam<std::string>("problem/code_generation_file_path");
  FileSystemUtil::createDirectoryIfNonexistent(file_path);
  cppad_jacobian_ptr_ = std::make_shared<autojac::SparseJacobianCodeGen<double>>(nr_of_cache_values, 1, func, file_path + "/cost_jacobian_library");

  // set gradient map
  unweightedCostResult().setExternalJacobianReference(cppad_jacobian_ptr_->getMap());
}

void SimpleCost::prepareCost() {}

void SimpleCost::updateValue() { calculateCostValue(getCcVector(), unweightedCostResult().values_); }

void SimpleCost::updateGradient() { cppad_jacobian_ptr_->updateJacobian(getCcVector()); }
}  // namespace test
}  // namespace optiminf
