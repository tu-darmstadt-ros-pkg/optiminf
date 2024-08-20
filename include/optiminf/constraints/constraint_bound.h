#pragma once

#include <limits>

namespace optiminf
{
class ConstraintBound
{
public:
  /**
   * @brief ConstraintBound represents the upper and / or lower boundary for a given constraint value.
   * @param lower_limit the value has to be larger or equal to this limit
   * @param upper_limit the value has to be smaller or equal to this limit
   */
  ConstraintBound(double lower_limit, double upper_limit);

  /**
   * @brief createUpperBound creates boundary with just an upper limit
   * @param upper_limit
   * @return
   */
  static ConstraintBound createUpperBound(double upper_limit);

  /**
   * @brief createLowerBound creates boundary with just a lower limit
   * @param lower_limit
   * @return
   */
  static ConstraintBound createLowerBound(double lower_limit);

  /**
   * @brief getLowerLimit returns the value of the lower limit
   * @return lower limit
   */
  double getLowerLimit() const;

  /**
   * @brief getUpperLimit returns the value of the upper limit
   * @return upper limit
   */
  double getUpperLimit() const;

  /**
   * @brief For lower bounds the value of the upper limit is set to INF_VALUE. For upper bounds the value of the lower limit is set to -INF_VALUE.
   */
  static constexpr double INF_VALUE = std::numeric_limits<double>::max();

private:
  double lower_limit_;
  double upper_limit_;
};
}  // namespace optiminf
