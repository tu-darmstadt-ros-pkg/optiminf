#pragma once

#include "optiminf/constraints/constraint_base.h"

namespace optiminf
{
namespace test
{
/**
 * @brief The SimpleConstraint class implements some constraints for testing purpose and their gradient calculation
 * @details The following constraints are implemented:
 * \f{eqnarray*}{
 *  x_0 &>& 1 \\
 *  x_1 &>& 2 \\
 *  x_2 &>& 1 \\
 *  \sqrt{{x_0}^2+x_1}+x_2 &>& 10
 * \f}
 */
class SimpleConstraint : public optiminf::ConstraintBase
{
public:
  explicit SimpleConstraint();

  virtual void setupConstraintImpl() final;

  virtual void prepareConstraint() final;

  virtual void updateValues() final;
  virtual void updateJacobian() final;
};
}  // namespace test
}  // namespace optiminf
