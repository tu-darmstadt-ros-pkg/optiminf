#include "gtest/gtest.h"

#include "optiminf/util/eigen_util.h"
#include "optiminf/splines/hermite_spline.h"

namespace optiminf
{
namespace test
{
void initOptVarTest(size_t dim, size_t nr_of_opt_variables, size_t nr_of_samples, double sample_delta_time, const std::vector<std::vector<double>>& values,
                    const std::vector<std::vector<bool>>& opt_vars, const std::vector<double>& expected_results)
{
  /* initialize nlp data */
  NlpData::Ptr nlp_data_ptr = std::make_shared<NlpData>();

  nlp_data_ptr->optimization_variables_.resize(nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.resize(nr_of_samples * dim * 2, nr_of_opt_variables);

  /* setup spline */
  HermiteSpline spline(nlp_data_ptr, dim);

  spline.prepareForSettingNodes(values.size(), nr_of_samples, nr_of_opt_variables, 0, 0, sample_delta_time);

  for (int i = 0; i < values.size(); i++)
  {
    spline.setNode(i, opt_vars[i], values[i]);
  }

  spline.finalizePreparation();

  for (size_t i = 0; i < nr_of_opt_variables; i++)
  {
    EXPECT_EQ(nlp_data_ptr->optimization_variables_(i), expected_results[i]);
  }
}

void updateValues(size_t dim, double result_tolerance, size_t nr_of_opt_variables, size_t nr_of_samples, double sample_delta_time, const std::vector<std::vector<double>>& values,
                  const std::vector<std::vector<bool>>& opt_vars, const Eigen::VectorXd& expected_polynom_result)
{
  /* initialize nlp data */
  NlpData::Ptr nlp_data_ptr = std::make_shared<NlpData>();

  nlp_data_ptr->optimization_variables_.resize(nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.resize(nr_of_samples * dim * 2, nr_of_opt_variables);

  /* setup spline */
  HermiteSpline spline(nlp_data_ptr, dim);

  spline.prepareForSettingNodes(values.size(), nr_of_samples, nr_of_opt_variables, 0, 0, sample_delta_time);

  for (int i = 0; i < values.size(); i++)
  {
    spline.setNode(i, opt_vars[i], values[i]);
  }

  spline.finalizePreparation();
  spline.updateValues();

  // check first and last sample point = first and last node
  for (size_t i = 0; i < dim * 2; i++)
  {
    EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(i), values[0][i], result_tolerance);
    EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_((nr_of_samples - 1) * dim * 2 + i), 0, result_tolerance);
  }

  ASSERT_EQ(expected_polynom_result.size(), dim * 2);
  for (size_t i = 0; i < dim * 2; i++)
  {
    EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(3 * dim * 2 + i), expected_polynom_result[i], result_tolerance);
  }
}

void updateJacobian(size_t dim, double result_tolerance, const std::vector<std::vector<double>>& values, const std::vector<std::vector<bool>>& opt_vars)
{
  /* initialize nlp data */
  NlpData::Ptr nlp_data_ptr = std::make_shared<NlpData>();

  size_t nr_of_opt_variables = 1;
  size_t nr_of_samples = 6;

  nlp_data_ptr->calculation_cache_.resize(nr_of_samples * dim * 2, nr_of_opt_variables);
  nlp_data_ptr->optimization_variables_.resize(nr_of_opt_variables);

  /* setup spline */
  HermiteSpline spline(nlp_data_ptr, dim);

  double sample_delta_time = 0.5;

  spline.prepareForSettingNodes(4, nr_of_samples, nr_of_opt_variables, 0, 0, sample_delta_time);

  for (int i = 0; i < values.size(); i++)
  {
    spline.setNode(i, opt_vars[i], values[i]);
  }

  spline.finalizePreparation();

  spline.updateJacobian();

  std::vector<Eigen::VectorXd> pos_before;
  std::vector<Eigen::VectorXd> vel_before;

  for (int i = 0; i < nr_of_samples; i++)
  {
    double sample_time = i * sample_delta_time;
    pos_before.push_back(spline.getValue(sample_time));
    vel_before.push_back(spline.getDerivative(sample_time));
  }

  // Forward Differences
  double delta = 0.0001;

  nlp_data_ptr->optimization_variables_(0) += delta;

  spline.updateValues();

  for (int i = 0; i < nr_of_samples; i++)
  {
    double sample_time = i * sample_delta_time;
    Eigen::VectorXd pos_dif = spline.getValue(sample_time) - pos_before[i];
    Eigen::VectorXd vel_dif = spline.getDerivative(sample_time) - vel_before[i];

    Eigen::VectorXd jac_pos_value = nlp_data_ptr->calculation_cache_.jacobian_map_.block(i * dim * 2, 0, dim, 1);
    Eigen::VectorXd jac_vel_value = nlp_data_ptr->calculation_cache_.jacobian_map_.block(i * dim * 2 + dim, 0, dim, 1);
    for (int j = 0; j < dim; j++)
    {
      EXPECT_NEAR(jac_pos_value(j), pos_dif(j) / delta, result_tolerance) << "Failed for sample " << i << " in dimension " << j;
      EXPECT_NEAR(jac_vel_value(j), vel_dif(j) / delta, result_tolerance) << "Failed for sample " << i << " in dimension " << j;
    }
  }
}

TEST(HermiteSplineTest, initOptVarDim1Test)
{
  double sample_delta_time = 0.5;
  size_t nr_of_samples = 6;

  std::vector<std::vector<double>> values_dim_1;
  std::vector<std::vector<bool>> opt_vars_dim_1;

  values_dim_1.push_back({ 0, 3, 1 });
  values_dim_1.push_back({ 1, 0.5, 0.7 });
  values_dim_1.push_back({ 4, 5.3, 0.8 });
  values_dim_1.push_back({ 0, 0, 0 });

  opt_vars_dim_1.push_back({ true, false, false });
  opt_vars_dim_1.push_back({ false, false, false });
  opt_vars_dim_1.push_back({ true, true, false });
  opt_vars_dim_1.push_back({ false, false, false });

  std::vector<double> expected_results_1 = { 0, 4, 5.3 };

  initOptVarTest(1, 3, nr_of_samples, sample_delta_time, values_dim_1, opt_vars_dim_1, expected_results_1);
}

TEST(HermiteSplineTest, initOptVarDim3Test)
{
  double sample_delta_time = 0.5;
  size_t nr_of_samples = 6;

  std::vector<std::vector<double>> values_dim_3;
  std::vector<std::vector<bool>> opt_vars_dim_3;

  values_dim_3.push_back({ 0, 0, 0, 0, 0, 0, 1 });
  values_dim_3.push_back({ 1, 0, 0, 2.7, 3.1, 0, 0.7 });
  values_dim_3.push_back({ 4, 5, 6, 0, 0, 0, 0.8 });
  values_dim_3.push_back({ 0, 0, 0, 0, 0, 0, 0 });

  opt_vars_dim_3.push_back({ false, false, false, false, false, false, false });
  opt_vars_dim_3.push_back({ false, false, true, false, true, false, false });
  opt_vars_dim_3.push_back({ true, true, true, false, false, false, false });
  opt_vars_dim_3.push_back({ false, false, false, false, false, false, false });

  std::vector<double> expected_results_3 = { 0, 3.1, 4, 5, 6 };

  initOptVarTest(3, 5, nr_of_samples, sample_delta_time, values_dim_3, opt_vars_dim_3, expected_results_3);
}

TEST(HermiteSplineTest, initOptVarDim6Test)
{
  double sample_delta_time = 0.5;
  size_t nr_of_samples = 6;

  std::vector<std::vector<double>> values_dim_6;
  std::vector<std::vector<bool>> opt_vars_dim_6;

  values_dim_6.push_back({ 0, 1, 7.4, 2.4, 1.2, 3, 6, 7, 4, 0, 1, 2.1, 1 });
  values_dim_6.push_back({ 1, 0.5, 4.4, 9.0, 1.5, 2.01, 1.3, 0.56, 4.2, 4.5, 3.9, 3, 0.7 });
  values_dim_6.push_back({ 4, 5.3, 0, 2.7, 3.1, 4, 5, 6, 0, 5, 1.8, 2, 0.8 });
  values_dim_6.push_back({ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });

  opt_vars_dim_6.push_back({ false, false, false, true, false, false, false, true, true, true, false, false, false });
  opt_vars_dim_6.push_back({ false, false, false, false, false, false, false, false, false, false, false, false, false });
  opt_vars_dim_6.push_back({ false, true, false, false, false, false, false, false, false, false, true, false, false });
  opt_vars_dim_6.push_back({ false, false, false, false, false, false, true, false, false, false, false, true, false });

  std::vector<double> expected_results_6 = { 2.4, 7, 4, 0, 5.3, 1.8, 0, 0 };

  initOptVarTest(6, 8, nr_of_samples, sample_delta_time, values_dim_6, opt_vars_dim_6, expected_results_6);
}

TEST(HermiteSplineTest, setupSparsityPatternTest)
{
  /* initialize nlp data */
  NlpData::Ptr nlp_data_ptr = std::make_shared<NlpData>();

  size_t nr_of_opt_variables = 5;
  size_t nr_of_samples = 6;

  nlp_data_ptr->optimization_variables_.resize(nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.values_.resize(nr_of_samples * 6);

  nlp_data_ptr->calculation_cache_.getInternalJacobian().resize(nr_of_samples * 6, nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.applyInternalStructureChangesToMap();

  /* setup spline */
  HermiteSpline spline(nlp_data_ptr, 3);

  double sample_delta_time =
      0.4999;  // when using 0.5 it whould be not clear in which polynom you end up when hitting exactly on one node. As there will be non zeros inserted for the other node of the
               // polynomil even if the values will be close to zero during calculation this is used to be able to predict the sparsity pattern

  spline.prepareForSettingNodes(4, nr_of_samples, nr_of_opt_variables, 0, 0, sample_delta_time);

  spline.setNode(0, { false, false, false, false, false, false, false }, { 0, 0, 0, 0, 0, 0, 1.0 });
  spline.setNode(1, { false, false, true, false, true, false, false }, { 1, 0, 0, 2.7, 3.1, 0, 0.7 });
  spline.setNode(2, { true, true, true, false, false, false, false }, { 4, 5, 6, 0, 0, 0, 0.8 });
  spline.setNode(3, { false, false, false, false, false, false, false }, { 0, 0, 0, 0, 0, 0, 0 });

  spline.finalizePreparation();

  ASSERT_EQ(nlp_data_ptr->calculation_cache_.jacobian_map_.nonZeros(),
            2 * (                  // each dimension has two entries one for position, one for velocity
                    2 * (3 + 1) +  // Node 1: opt vars in two dimmensions affecting 3 samples in polynom 1 and 1 in polinom 2
                    3 * (1 + 2)    // Node 2: opt vars in three dimmensions affecting 1 samples in polynom 2 and 2 in polinom 3
                    ));
  Eigen::MatrixXd expected_pattern = EigenUtil::loadEigenMatrixFromCsv<Eigen::MatrixXd>(SOURCE_DIR "/test/test_data/HermiteSplineTest_sparsity_pattern.csv");
  EigenUtil::setAllNonZerosToValue(nlp_data_ptr->calculation_cache_.getInternalJacobian(), 1);
  EXPECT_TRUE(nlp_data_ptr->calculation_cache_.jacobian_map_.isApprox(expected_pattern));
}

TEST(HermiteSplineTest, updateValuesAfterChangedOptVarsTest)
{
  /* update opt var and check for result */

  const double result_tolerance = 0.0001;

  /* initialize nlp data */
  NlpData::Ptr nlp_data_ptr = std::make_shared<NlpData>();

  size_t nr_of_opt_variables = 5;
  size_t nr_of_samples = 6;

  nlp_data_ptr->optimization_variables_.resize(nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.values_.resize(nr_of_samples * 6);

  nlp_data_ptr->calculation_cache_.getInternalJacobian().resize(nr_of_samples * 6, nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.applyInternalStructureChangesToMap();

  /* setup spline */
  HermiteSpline spline(nlp_data_ptr, 3);

  double sample_delta_time = 0.5;
  spline.prepareForSettingNodes(4, nr_of_samples, nr_of_opt_variables, 0, 0, sample_delta_time);

  spline.setNode(0, { false, true, false, false, false, false, false }, { 1, 2, 3, 4, 5, 6, 1.0 });
  spline.setNode(1, { false, false, true, false, false, false, false }, { 6, 5, 4, 3, 2, 1, 1.2 });
  spline.setNode(2, { true, true, true, false, false, false, false }, { 2, 4, 6, 8, 10, 11, 0.3 });
  spline.setNode(3, { false, false, false, false, false, false, false }, { 0, 0, 0, 0, 0, 0, 0 });

  spline.finalizePreparation();
  spline.updateValues();

  nlp_data_ptr->optimization_variables_(1) = 1.5;
  spline.updateValues();

  EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(12), 6, result_tolerance);
  EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(13), 5, result_tolerance);
  EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(14), 1.5, result_tolerance);
  EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(15), 3, result_tolerance);
  EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(16), 2, result_tolerance);
  EXPECT_NEAR(nlp_data_ptr->calculation_cache_.values_(17), 1, result_tolerance);
}

TEST(HermiteSplineTest, updateValuesDim1Test)
{
  double sample_delta_time =
      0.5;  // when using 0.5 it whould be not clear in which polynom you end up when hitting exactly on one node. As there will be non zeros inserted for the other node of the
            // polynomil even if the values will be close to zero during calculation this is used to be able to predict the sparsity pattern

  size_t nr_of_samples = 6;

  size_t dim = 1;
  size_t nr_of_opt_variables = 3;

  std::vector<std::vector<double>> values_dim_1;
  std::vector<std::vector<bool>> opt_vars_dim_1;

  values_dim_1.push_back({ 1, 2, 1.0 });
  values_dim_1.push_back({ 6, 5, 1.2 });
  values_dim_1.push_back({ 2, 4, 0.3 });
  values_dim_1.push_back({ 0, 0, 0, 0, 0, 0, 0 });

  opt_vars_dim_1.push_back({ false, true, false });
  opt_vars_dim_1.push_back({ false, false, false });
  opt_vars_dim_1.push_back({ true, true, false });
  opt_vars_dim_1.push_back({ false, false, false });

  std::vector<double> polynom_input_1 = { 6, 5, 1.2, 2, 4 };
  Eigen::VectorXd expected_polynom_result_1(dim * 2);
  HermiteSpline::calcPolynomialValues<double>(Eigen::Map<Eigen::VectorXd>(polynom_input_1.data(), dim * 4 + 1, 1), 0.5, dim, expected_polynom_result_1);

  updateValues(dim, 0.0001, nr_of_opt_variables, nr_of_samples, sample_delta_time, values_dim_1, opt_vars_dim_1, expected_polynom_result_1);
}

TEST(HermiteSplineTest, updateValuesDim3Test)
{
  double sample_delta_time =
      0.5;  // when using 0.5 it whould be not clear in which polynom you end up when hitting exactly on one node. As there will be non zeros inserted for the other node of the
            // polynomil even if the values will be close to zero during calculation this is used to be able to predict the sparsity pattern

  size_t nr_of_samples = 6;

  /* 3 dimensions */
  size_t dim = 3;
  size_t nr_of_opt_variables = 5;

  std::vector<std::vector<double>> values_dim_3;
  std::vector<std::vector<bool>> opt_vars_dim_3;

  values_dim_3.push_back({ 1, 2, 3, 4, 5, 6, 1.0 });
  values_dim_3.push_back({ 6, 5, 4, 3, 2, 1, 1.2 });
  values_dim_3.push_back({ 2, 4, 6, 8, 10, 11, 0.3 });
  values_dim_3.push_back({ 0, 0, 0, 0, 0, 0, 0 });

  opt_vars_dim_3.push_back({ false, true, false, false, false, false, false });
  opt_vars_dim_3.push_back({ false, false, true, false, false, false, false });
  opt_vars_dim_3.push_back({ true, true, true, false, false, false, false });
  opt_vars_dim_3.push_back({ false, false, false, false, false, false, false });

  /* directly calculate values for sample 4 at t=1.5s and compare with result*/
  std::vector<double> polynom_input_3 = { 6, 5, 4, 3, 2, 1, 1.2, 2, 4, 6, 8, 10, 11 };
  Eigen::VectorXd expected_polynom_result_3(dim * 2);
  HermiteSpline::calcPolynomialValues<double>(Eigen::Map<Eigen::VectorXd>(polynom_input_3.data(), dim * 4 + 1, 1), 0.5, dim, expected_polynom_result_3);

  updateValues(dim, 0.0001, nr_of_opt_variables, nr_of_samples, sample_delta_time, values_dim_3, opt_vars_dim_3, expected_polynom_result_3);
}

TEST(HermiteSplineTest, updateValuesDim6Test)
{
  double sample_delta_time =
      0.5;  // when using 0.5 it whould be not clear in which polynom you end up when hitting exactly on one node. As there will be non zeros inserted for the other node of the
            // polynomil even if the values will be close to zero during calculation this is used to be able to predict the sparsity pattern

  size_t nr_of_samples = 6;

  std::vector<std::vector<double>> values_dim_6;
  std::vector<std::vector<bool>> opt_vars_dim_6;

  size_t dim = 6;
  size_t nr_of_opt_variables = 8;

  values_dim_6.push_back({ 0, 1, 7.4, 2.4, 1.2, 3, 6, 7, 4, 0, 1, 2.1, 1.0 });
  values_dim_6.push_back({ 1, 0.5, 4.4, 9.0, 1.5, 2.01, 1.3, 0.56, 4.2, 4.5, 3.9, 3, 1.2 });
  values_dim_6.push_back({ 4, 5.3, 0, 2.7, 3.1, 4, 5, 6, 0, 5, 1.8, 2, 0.3 });
  values_dim_6.push_back({ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });

  opt_vars_dim_6.push_back({ false, false, false, true, false, false, false, true, true, true, false, false, false });
  opt_vars_dim_6.push_back({ false, false, false, false, false, false, false, false, false, false, false, false, false });
  opt_vars_dim_6.push_back({ false, true, false, false, false, false, false, false, false, false, true, false, false });
  opt_vars_dim_6.push_back({ false, false, false, false, false, false, true, false, false, false, false, true, false });

  std::vector<double> polynom_input_6 = { 1, 0.5, 4.4, 9.0, 1.5, 2.01, 1.3, 0.56, 4.2, 4.5, 3.9, 3, 1.2, 4, 5.3, 0, 2.7, 3.1, 4, 5, 6, 0, 5, 1.8, 2 };
  Eigen::VectorXd expected_polynom_result_6(dim * 2);

  HermiteSpline::calcPolynomialValues<double>(Eigen::Map<Eigen::VectorXd>(polynom_input_6.data(), dim * 4 + 1, 1), 0.5, dim, expected_polynom_result_6);

  updateValues(dim, 0.0001, nr_of_opt_variables, nr_of_samples, sample_delta_time, values_dim_6, opt_vars_dim_6, expected_polynom_result_6);
}

TEST(HermiteSplineTest, getValueAndVelocityTest)
{
  const double result_tolerance = 0.0001;

  /* initialize nlp data */
  NlpData::Ptr nlp_data_ptr = std::make_shared<NlpData>();

  size_t nr_of_opt_variables = 5;
  size_t nr_of_samples = 6;

  nlp_data_ptr->optimization_variables_.resize(nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.values_.resize(nr_of_samples * 6);

  nlp_data_ptr->calculation_cache_.getInternalJacobian().resize(nr_of_samples * 6, nr_of_opt_variables);
  nlp_data_ptr->calculation_cache_.applyInternalStructureChangesToMap();

  /* setup spline */
  HermiteSpline spline(nlp_data_ptr, 3);

  double sample_delta_time =
      0.5;  // when using 0.5 it whould be not clear in which polynom you end up when hitting exactly on one node. As there will be non zeros inserted for the other node of the
            // polynomil even if the values will be close to zero during calculation this is used to be able to predict the sparsity pattern

  spline.prepareForSettingNodes(4, nr_of_samples, nr_of_opt_variables, 0, 0, sample_delta_time);

  spline.setNode(0, { false, true, false, false, false, false, false }, { 1, 2, 3, 4, 5, 6, 1.0 });
  spline.setNode(1, { false, false, true, false, false, false, false }, { 6, 5, 4, 3, 2, 1, 1.2 });
  spline.setNode(2, { true, true, true, false, false, false, false }, { 2, 4, 6, 8, 10, 11, 0.3 });
  spline.setNode(3, { false, false, false, false, false, false, false }, { 0, 0, 0, 0, 0, 0, 0 });

  spline.finalizePreparation();

  // beginning of the spline
  Eigen::Vector3d pos1 = spline.getValue(0.0);
  Eigen::Vector3d vel1 = spline.getDerivative(0.0);

  EXPECT_NEAR(pos1(0), 1, result_tolerance);
  EXPECT_NEAR(pos1(1), 2, result_tolerance);
  EXPECT_NEAR(pos1(2), 3, result_tolerance);

  EXPECT_NEAR(vel1(0), 4, result_tolerance);
  EXPECT_NEAR(vel1(1), 5, result_tolerance);
  EXPECT_NEAR(vel1(2), 6, result_tolerance);

  // next point
  Eigen::Vector3d pos2 = spline.getValue(1.0);
  Eigen::Vector3d vel2 = spline.getDerivative(1.0);

  EXPECT_NEAR(pos2(0), 6, result_tolerance);
  EXPECT_NEAR(pos2(1), 5, result_tolerance);
  EXPECT_NEAR(pos2(2), 4, result_tolerance);

  EXPECT_NEAR(vel2(0), 3, result_tolerance);
  EXPECT_NEAR(vel2(1), 2, result_tolerance);
  EXPECT_NEAR(vel2(2), 1, result_tolerance);

  // between two points
  Eigen::Vector3d pos3 = spline.getValue(1.3);
  Eigen::Vector3d vel3 = spline.getDerivative(1.3);

  std::vector<double> polynom_input = { 6, 5, 4, 3, 2, 1, 1.2, 2, 4, 6, 8, 10, 11 };

  /* directly calculate values for the value between two points for t=1.3s */
  Eigen::VectorXd expected_polynom_value(3 * 2);

  HermiteSpline::calcPolynomialValues<double>(Eigen::Map<Eigen::VectorXd>(polynom_input.data(), 13, 1), 0.3, 3, expected_polynom_value);

  ASSERT_EQ(expected_polynom_value.size(), 6);

  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_NEAR(pos3(i), expected_polynom_value(i), result_tolerance);
    EXPECT_NEAR(vel3(i), expected_polynom_value(3 + i), result_tolerance);
  }
}

TEST(HermiteSplineTest, updateJacobianDim1Test)
{
  std::vector<std::vector<double>> values_dim_1;
  std::vector<std::vector<bool>> opt_vars_dim_1;

  values_dim_1.push_back({ 1, 0, 0.999 });
  values_dim_1.push_back({ 1, 0, 0.7 });
  values_dim_1.push_back({ 4, 5, 0.81 });
  values_dim_1.push_back({ 0, 0, 0 });

  opt_vars_dim_1.push_back({ false, false, false });
  opt_vars_dim_1.push_back({ false, true, false });
  opt_vars_dim_1.push_back({ false, false, false });
  opt_vars_dim_1.push_back({ false, false, false });

  updateJacobian(1, 0.0001, values_dim_1, opt_vars_dim_1);
}

TEST(HermiteSplineTest, updateJacobianDim3Test)
{
  std::vector<std::vector<double>> values_dim_3;
  std::vector<std::vector<bool>> opt_vars_dim_3;

  values_dim_3.push_back({ 1, 0, 0, 0, 0, 0, 0.999 });
  values_dim_3.push_back({ 1, 0, 3, 2.7, 3.1, 0, 0.7 });
  values_dim_3.push_back({ 4, 5, 6, 0, 0, 0, 0.81 });
  values_dim_3.push_back({ 0, 0, 0, 0, 0, 0, 0 });

  opt_vars_dim_3.push_back({ false, false, false, false, false, false, false });
  opt_vars_dim_3.push_back({ false, false, true, false, false, false, false });
  opt_vars_dim_3.push_back({ false, false, false, false, false, false, false });
  opt_vars_dim_3.push_back({ false, false, false, false, false, false, false });

  updateJacobian(3, 0.0001, values_dim_3, opt_vars_dim_3);
}

TEST(HermiteSplineTest, updateJacobianDim6Test)
{
  std::vector<std::vector<double>> values_dim_6;
  std::vector<std::vector<bool>> opt_vars_dim_6;

  opt_vars_dim_6.push_back({ false, false, false, false, false, false, false, false, false, false, false, false, false });
  opt_vars_dim_6.push_back({ false, false, false, false, false, false, false, false, false, false, false, false, false });
  opt_vars_dim_6.push_back({ false, false, false, false, false, false, false, false, false, false, true, false, false });
  opt_vars_dim_6.push_back({ false, false, false, false, false, false, false, false, false, false, false, false, false });

  values_dim_6.push_back({ 0, 1, 7.4, 2.4, 1.2, 3, 6, 7, 4, 0, 1, 2.1, 0.999 });
  values_dim_6.push_back({ 1, 0.5, 4.4, 9.0, 1.5, 2.0, 1.3, 0.5, 4.2, 4.5, 3.9, 3, 0.7 });
  values_dim_6.push_back({ 4, 5.3, 0, 2.7, 3.1, 4, 5, 6, 0, 5, 1.8, 2, 0.81 });
  values_dim_6.push_back({ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });

  updateJacobian(1, 0.0001, values_dim_6, opt_vars_dim_6);
}
}  // namespace test
}  // namespace optiminf
