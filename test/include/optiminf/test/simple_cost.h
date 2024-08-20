#pragma once

#include "optiminf/costs/cost_base.h"

#include "optiminf/test/simple_cc_view.h"

#include "autojac/sparse_jacobian_code_gen.h"
namespace optiminf
{
namespace test
{
/**
 * @brief The SimpleCost class implements the cost calculation in calculateCostValue() and uses the cppad interface for computing the gradient
 */
class SimpleCost : public optiminf::CostBase
{
public:
  explicit SimpleCost();

  void setupCostImpl() final;
  void prepareCost() final;
  void updateValue() final;
  void updateGradient() final;

  /**
   * @brief calculateCostValue \f$x_0+{x_1}^2+x_2 \f$.
   * @param input
   * @param output
   */
  template <typename Scalar>
  static void calculateCostValue(const VectorXS<Scalar>& input, VectorXS<Scalar>& output)
  {
    SimpleCcView cc_view(input);
    output(0) = cc_view.x0() + cc_view.x1Square() + cc_view.x2();
  }

private:
  autojac::SparseJacobianCodeGen<double>::Ptr cppad_jacobian_ptr_;
};
}  // namespace test
}  // namespace optiminf
