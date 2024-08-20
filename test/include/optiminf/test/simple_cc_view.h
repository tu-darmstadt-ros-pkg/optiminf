#pragma once

#include "optiminf/data_structs/nlp_data.h"

namespace optiminf
{
namespace test
{
template <typename EigenVector>
class SimpleCcView
{
public:
  /**
   * @brief SimpleCcView This view enables to access the calculations stored in the calculation cache (cc) by names instead of using their index
   * @details This way of accessing the variables is less error prone than using the index. The concept of using views also enables to experiment with different vector layouts to
   * optimize for performance (i.e. memory cache hits) while having to adapt the indexing scheme only within this class and not in every cost and constraint using the cc.
   * @param cc_vector for which the view is created. If this vector is of a non const type the accessors within the class can be used for reading and writing values, otherwise they
   * are read only
   * @tparam Eigen vector type. If the type is const the accessors are read only
   */
  SimpleCcView(EigenVector& cc_vector)
    : cc_vector_(cc_vector)
  {}

  // ensure return type has the const qualifier when EigenVector has a const qualifier in order to be able to return a reference
  using ScalarReturnType = typename std::conditional<std::is_const<EigenVector>::value, const typename EigenVector::Scalar&, typename EigenVector::Scalar&>::type;

  ScalarReturnType x0() { return cc_vector_(0); }
  ScalarReturnType x1() { return cc_vector_(1); }
  ScalarReturnType x2() { return cc_vector_(2); }

  ScalarReturnType x0Square() { return cc_vector_(3); }
  ScalarReturnType x1Square() { return cc_vector_(4); }
  ScalarReturnType x2Square() { return cc_vector_(5); }

  /**
   * @brief simpleCalc  access to the field where the following value is stored: \f$ \sqrt{1 + {x_0}^2+x_1}+x_2    \f$
   * @return
   */
  ScalarReturnType simpleCalc() { return cc_vector_(6); }

private:
  EigenVector& cc_vector_;
};
}  // namespace test
}  // namespace optiminf
