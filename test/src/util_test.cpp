#include "gtest/gtest.h"

#include "optiminf/util/eigen_util.h"

namespace optiminf
{
namespace test
{
TEST(UtilTest, SparseMapTest)
{
  Eigen::SparseMatrix<double, Eigen::RowMajor> rm_matrix(7, 9);

  /* fill with test values */
  rm_matrix.coeffRef(1, 5) = 1;
  rm_matrix.coeffRef(2, 8) = 2;
  rm_matrix.coeffRef(6, 8) = 3;

  Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor>> result = EigenUtil::getMapFromMatrix(rm_matrix);

  ASSERT_EQ(rm_matrix.rows(), result.rows());
  ASSERT_EQ(rm_matrix.cols(), result.cols());

  for (size_t i = 0; i < rm_matrix.cols(); i++)
  {
    for (size_t j = 0; j < rm_matrix.rows(); j++)
    {
      ASSERT_EQ(rm_matrix.coeff(j, i), result.coeff(j, i));
    }
  }
}
}  // namespace test
}  // namespace optiminf
