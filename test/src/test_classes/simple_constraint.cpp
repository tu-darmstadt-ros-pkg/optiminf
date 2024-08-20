#include "optiminf/test/simple_constraint.h"

#include "optiminf/util/eigen_util.h"
#include "optiminf/test/simple_cc_view.h"

namespace optiminf
{
namespace test
{
SimpleConstraint::SimpleConstraint()
  : optiminf::ConstraintBase("ExampleConstraint")
{}

void SimpleConstraint::setupConstraintImpl()
{
  resizeConstraintData(4);

  // this is an example of providing a custom coded jacobian instead of using autodiff
  constraintValueResult().getInternalJacobian().coeffRef(0, 0) = 1;
  constraintValueResult().getInternalJacobian().coeffRef(1, 1) = 1;
  constraintValueResult().getInternalJacobian().coeffRef(2, 2) = 1;
  constraintValueResult().getInternalJacobian().coeffRef(3, 6) = 1;
  constraintValueResult().applyInternalStructureChangesToMap();

  // set bounds
  lowerBounds() = Eigen::VectorXd::Ones(4);
  lowerBounds()(1) = 2;
  lowerBounds()(3) = 10;

  upperBounds() = Eigen::VectorXd::Ones(4) * std::numeric_limits<double>::max();
}

void SimpleConstraint::prepareConstraint() {}

void SimpleConstraint::updateValues()
{
  SimpleCcView cc_view(getCCVector());

  constraintValueResult().values_(0) = cc_view.x0();
  constraintValueResult().values_(1) = cc_view.x1();
  constraintValueResult().values_(2) = cc_view.x2();
  constraintValueResult().values_(3) = cc_view.simpleCalc();
}

void SimpleConstraint::updateJacobian() {}
}  // namespace test
}  // namespace optiminf
