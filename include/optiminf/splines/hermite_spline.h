#pragma once
#include "gtest/gtest.h"

#include "optiminf/util/eigen_util.h"
#include "optiminf/data_structs/nlp_data.h"

namespace optiminf
{
// forward declare test class for declaring FRIEND_TEST
namespace test
{
class HermiteSplineTest;
class HermiteSplineTest_updateValuesDim1Test_Test;
class HermiteSplineTest_updateValuesDim3Test_Test;
class HermiteSplineTest_updateValuesDim6Test_Test;
class HermiteSplineTest_getValueAndVelocityTest_Test;
}  // namespace test

class HermiteSpline
{
public:
  using Ptr = std::shared_ptr<HermiteSpline>;
  using ConstPtr = std::shared_ptr<const HermiteSpline>;

  /**
   * @brief HermiteSpline
   * @param nlp_data_ptr
   * @param dim dimensionality of the spline: each node of the spline has 2n+1 values
   */
  explicit HermiteSpline(NlpData::Ptr nlp_data_ptr, size_t dim);

  /**
   * @brief prepareSpline sets the member variables and allocates memory for nodes. Thus this function needs to be called before setNode()
   * @param number_of_spline_nodes number of nodes that represent the start and end nodes for all polynomials
   * @param nr_of_samples number of equidistant samples of the spline that should be added to the calculation cache
   * @param nr_of_optimization_variables number of optimization variables that are used for the spline
   * @param opt_var_start_idx index of the first optimization varibale in nlp_data that represents a node value of this spline
   * @param calculation_cache_start_idx index of the first element in the calculation cache vector in nlp_data that represents a sample
   * @param sample_delta_time the time interval between each sample
   */
  void prepareForSettingNodes(size_t nr_of_spline_nodes, size_t nr_of_samples, size_t nr_of_optimization_variables, size_t opt_var_start_idx, size_t calculation_cache_start_idx,
                              double sample_delta_time);

  /**
   * @brief setNode sets the position and velocity values of the node as well as the polynomial duration.
   * @details This function needs to be called after prepareForSettingNodes(). Call finalizePreparation() when all nodes have been set. For a three dimensional spline the
   * node_values vector would have the following order [x, y, z, xd, yd, zd, T] where x, y, z are equal to the spline values at the node, xd, yd and zd are equal to the first
   derivative and T represents the duration of the polynom defined by this node and the node that is added next. Hence the value of T for the last node that is added before calling
   finalizePreparation() does not have an effect.
   * @param node_idx spline node index
   * @param optimization_variable_flags vector of boolean to inicate which value should be optimized
   * @param node_values the values vector should first contain the values, than the first derivatives and the duration the polynom that is defined by this and the following node as
   * last element
   */
  void setNode(size_t node_idx, const std::vector<bool>& optimization_variable_flags, const std::vector<double>& node_values);

  /**
   * @brief finalizePreparation performs all necessary steps that need to be done before the optimization can be started afer all nodes have been set.
   * @details This function initializes the optimization variables in the NlpData struct and adds non zeros to the calculation cache jacobian.
   * @warning In oder to call this function the jacobian of the calculation cache and the vector containing the optimization variables in NlpData have to be resized s.t. all
   * entries that have to be set are available. The function getNumberOfOptVars() can be used to querry the number of optimization variables needed by this spline.
   */
  void finalizePreparation();

  /**
   * @brief getNumberOfOptVars returns the number of optimization variables used after all nodes have been added by setNode()
   * @return number of optimization variables used by the spline
   */
  size_t getNumberOfOptVars();

  /**
   * @brief getNumberOfSampleValues returns the number of values that represent the spline values and its first derivatives with respect to time for all spline dimensions and all
   * samples
   * @return
   */
  size_t getNumberOfSampleValues();

  /**
   * @brief updateValues updates the values for all samples in the calculation cache
   */
  void updateValues();

  /**
   * @brief getValue returns the value of the spline at time t
   * @param t the time should be greater or equal zero and smaller or equal to the sum of all polynomial durations, i.e. the sum of the values for T of all but the last node that
   * has been added
   * @return the n-dimensional value of the spline at time t
   */
  Eigen::VectorXd getValue(double t);

  /**
   * @brief getValue returns the first derivative with respect to time of the spline at time t
   * @param t the time should be greater or equal zero and smaller or equal to the sum of all polynomial durations, i.e. the sum of the values for T of all but the last node that
   * has been added
   * @return the n-dimensional value of the first derivative of the spline at time t
   */
  Eigen::VectorXd getDerivative(double t);

  /**
   * @brief updateJacobian updates the jacobian values for all samples in the calculation cache
   */
  void updateJacobian();

private:
  void initOptVars();

  void setupJacobianSparsityPattern();

  void copyOptVars();

  FRIEND_TEST(test::HermiteSplineTest, updateValuesDim1Test);
  FRIEND_TEST(test::HermiteSplineTest, updateValuesDim3Test);
  FRIEND_TEST(test::HermiteSplineTest, updateValuesDim6Test);
  FRIEND_TEST(test::HermiteSplineTest, getValueAndVelocityTest);

  /**
   * @brief calcPolynomialValues
   * @param node_values values of the start and end node of the polinomial
   * @param local_time time at which the polinomial should be evaluated
   * @param nr_of_dim dimension of the spline
   * @param polynomial_values the result will be written to this expression
   * @warning polynomial_values is not resized for performance reasons make sure to have it resizeded to 2*nr_of_dim before calling this function
   */
  template <typename Scalar>
  static void calcPolynomialValues(const Eigen::Ref<const VectorXS<Scalar>>& node_values, const Scalar& local_time, size_t nr_of_dim,
                                   Eigen::Ref<VectorXS<Scalar>> polynomial_values)
  {
    const VectorXS<Scalar>& p0 = node_values.segment(0, nr_of_dim);
    const VectorXS<Scalar>& v0 = node_values.segment(nr_of_dim, nr_of_dim);
    // skip delta time value
    const VectorXS<Scalar>& p1 = node_values.segment(2 * nr_of_dim + 1, nr_of_dim);
    const VectorXS<Scalar>& v1 = node_values.segment(3 * nr_of_dim + 1, nr_of_dim);

    const Scalar polyonom_duration = node_values(2 * nr_of_dim);

    const VectorXS<Scalar>& a = p0;
    const VectorXS<Scalar>& b = v0;
    const VectorXS<Scalar>& c = -(3 * p0 - 3 * p1 + 2 * polyonom_duration * v0 + polyonom_duration * v1) / (polyonom_duration * polyonom_duration);
    const VectorXS<Scalar>& d = (2 * p0 - 2 * p1 + polyonom_duration * v0 + polyonom_duration * v1) / (polyonom_duration * polyonom_duration * polyonom_duration);

    const Scalar t2 = local_time * local_time;
    const Scalar t3 = t2 * local_time;

    polynomial_values.head(nr_of_dim) = a + local_time * b + t2 * c + t3 * d;
    polynomial_values.tail(nr_of_dim) = b + local_time * 2 * c + 3 * t2 * d;
  }

  /**
   * @brief findPolynom finds the polynom that containts given time point and
   * sets the respective current node index and polynomial start, end time.
   * */
  void findPolynom(double time);

  /**
   * @brief getPolynomialValuesAtTime calculates polynomial value at the given time point
   * @param time time at which the spline should be evaluated
   * @param spline_values the result will be written to this expression
   * warning spline_values is not resized for performance reasons make sure to have it resizeded to 2*nr_of_dim before calling this function
   */
  template <typename Scalar>
  void calculateSplineValuesAtTime(double time, Eigen::Ref<VectorXS<Scalar>> spline_values)
  {
    findPolynom(time);

    calcPolynomialValues<Scalar>(spline_input_variables_.segment(current_node_idx_ * nr_of_variables_per_node_, nr_of_variables_start_and_end_node_),
                                 time - current_polynomial_start_time_, nr_of_dim_, std::move(spline_values));
  }

  void updatePolynomialJacobian(size_t start_node_idx, double local_time);

  size_t getSplineInputVariableStartIndex(size_t node_idx);

  Eigen::VectorXd spline_input_variables_;
  std::vector<int> opt_var_to_input_variable_idx_;

  const size_t nr_of_dim_;
  // number of variables of the start and end node of a polynom without the delta time in the end node
  const size_t nr_of_variables_start_and_end_node_;
  const size_t nr_of_variables_per_node_;

  double sample_delta_time_ = 0;

  size_t nr_of_spline_nodes_ = 0;
  size_t nr_of_samples_ = 0;

  // last calculated polynomial start, end times and the respective node index
  // initialize s.t. they get set correctly by the first call of findPolynom()
  double current_polynomial_start_time_ = std::numeric_limits<double>::max();
  double current_polynomial_end_time_ = std::numeric_limits<double>::max();
  size_t current_node_idx_ = 0;

  size_t opt_var_start_idx_ = 0;
  size_t calculation_cache_start_idx_ = 0;

  NlpData::Ptr nlp_data_ptr_;

  ConstDenseJacobianMap<double> jacobian_map_;
  Eigen::MatrixXd polynomial_jacobian_;
  Eigen::VectorXd jac_input_temp_;
};
}  // namespace optiminf
