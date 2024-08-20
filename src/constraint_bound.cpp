#include "optiminf/constraints/constraint_bound.h"

namespace optiminf
{
ConstraintBound::ConstraintBound(double lower_limit, double upper_limit)
  : lower_limit_(lower_limit)
  , upper_limit_(upper_limit)
{}

ConstraintBound ConstraintBound::createUpperBound(double upper_limit) { return ConstraintBound(-INF_VALUE, upper_limit); }

ConstraintBound ConstraintBound::createLowerBound(double lower_limit) { return ConstraintBound(lower_limit, INF_VALUE); }

double ConstraintBound::getLowerLimit() const { return lower_limit_; }

double ConstraintBound::getUpperLimit() const { return upper_limit_; }
}  // namespace optiminf
