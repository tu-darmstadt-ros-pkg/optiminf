#pragma once

#include "optiminf/calculation_cache/calculation_cache_manager_base.h"
#include "optiminf/test/simple_cc_view.h"

#include "autojac/sparse_jacobian_code_gen.h"

namespace optiminf
{
namespace test
{
/**
 * @brief The calculation cache of this class consists of seven values based on three optimization variables and uses auto diff for calculatin the gradient.
 * @details The calculation cache values are given in the following order:
 *  \f{eqnarray*}{
 *  cc_0 &:=& x_0 \\
 *  cc_1 &:=& x_1 \\
 *  cc_2 &:=& x_2 \\
 *  cc_3 &:=& x_0^2 \\
 *  cc_4 &:=& x_1^2 \\
 *  cc_5 &:=& x_2^2 \\
 *  cc_6 &:=& \sqrt{{x_0}^2+x_1}+x_2
 * \f}
 */
class SimpleCalculationCache : public optiminf::CalculationCacheManagerBase
{
public:
  SimpleCalculationCache(optiminf::ManagerContainer::ConstPtr manager_container_ptr, optiminf::NlpData::ConstPtr nlp_data_ptr);

  void setupCalculationCache() final;
  void prepareCalculationCache() final;

  void updateValues() final;
  void updateJacobian() final;

private:
  template <typename Scalar>
  static void calculateCacheValues(const VectorXS<Scalar>& input, VectorXS<Scalar>& output)
  {
    SimpleCcView cc_view(output);

    cc_view.x0() = input(0);
    cc_view.x1() = input(1);
    cc_view.x2() = input(2);

    cc_view.x0Square() = input(0) * input(0);
    cc_view.x1Square() = input(1) * input(1);
    cc_view.x2Square() = input(2) * input(2);

    cc_view.simpleCalc() = sqrt(cc_view.x0Square() + cc_view.x1()) + cc_view.x2();
  }

  autojac::SparseJacobianCodeGen<double>::Ptr cppad_jacobian_ptr_;
};
}  // namespace test
}  // namespace optiminf
