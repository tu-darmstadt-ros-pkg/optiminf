#include "optiminf/splines/hermite_spline.h"

namespace optiminf
{
HermiteSpline::HermiteSpline(NlpData::Ptr nlp_data_ptr, size_t dim)
  : jacobian_map_(nullptr, 0, 0)
  , nlp_data_ptr_(std::move(nlp_data_ptr))
  , nr_of_dim_(dim)
  , nr_of_variables_per_node_(dim * 2 + 1)
  , nr_of_variables_start_and_end_node_(dim * 4 + 1)
{
  jac_input_temp_ = Eigen::VectorXd::Ones(nr_of_variables_per_node_ * 2, 1);
  polynomial_jacobian_ = Eigen::MatrixXd::Zero(2 * nr_of_dim_, 4 * nr_of_dim_ + 2);
}

void HermiteSpline::prepareForSettingNodes(size_t nr_of_spline_nodes, size_t nr_of_samples, size_t nr_of_optimization_variables, size_t opt_var_start_idx,
                                           size_t calculation_cache_start_idx, double sample_delta_time)
{
  spline_input_variables_.resize(nr_of_spline_nodes * nr_of_variables_per_node_);
  nr_of_spline_nodes_ = nr_of_spline_nodes;
  opt_var_start_idx_ = opt_var_start_idx;
  nr_of_samples_ = nr_of_samples;
  sample_delta_time_ = sample_delta_time;
  calculation_cache_start_idx_ = calculation_cache_start_idx;

  opt_var_to_input_variable_idx_.reserve(nr_of_optimization_variables);
  // in case multiple problems are solved sequentially make sure the opt_var_to_input_variable_idx_ is empty everytime before adding the nodes
  opt_var_to_input_variable_idx_.resize(0);
}

void HermiteSpline::updateValues()
{
  copyOptVars();

  for (size_t i = 0; i < nr_of_samples_; i++)
  {
    double sample_time = i * sample_delta_time_;

    size_t segment_start_idx = calculation_cache_start_idx_ + i * (nr_of_variables_per_node_ - 1);
    calculateSplineValuesAtTime<double>(sample_time, nlp_data_ptr_->calculation_cache_.values_.segment(segment_start_idx, nr_of_variables_per_node_ - 1));
  }
}

Eigen::VectorXd HermiteSpline::getValue(double time)
{
  Eigen::VectorXd ret(nr_of_dim_ * 2);
  calculateSplineValuesAtTime<double>(time, ret);
  return ret.segment(0, nr_of_dim_);
}

Eigen::VectorXd HermiteSpline::getDerivative(double time)
{
  Eigen::VectorXd ret(nr_of_dim_ * 2);
  calculateSplineValuesAtTime<double>(time, ret);
  return ret.segment(nr_of_dim_, nr_of_dim_);
}

void HermiteSpline::updateJacobian()
{
  copyOptVars();

  for (size_t i = 0; i < nr_of_samples_; i++)
  {
    double sample_time = i * sample_delta_time_;

    findPolynom(sample_time);

    updatePolynomialJacobian(current_node_idx_, sample_time - current_polynomial_start_time_);

    size_t row_start_idx = calculation_cache_start_idx_ + i * (nr_of_variables_per_node_ - 1);

    for (size_t j = 0; j < (nr_of_variables_per_node_ - 1); j++)
    {
      Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(nlp_data_ptr_->calculation_cache_.getInternalJacobian(), row_start_idx + j);

      for (; it; ++it)
      {
        // this uses iterators instead of coeffReff for performance reasons
        // for details have a look here: https://eigen.tuxfamily.org/dox/group__TutorialSparse.html#TutorialSparseFilling
        it.valueRef() = polynomial_jacobian_(j, opt_var_to_input_variable_idx_[it.col() - opt_var_start_idx_] - current_node_idx_ * nr_of_variables_per_node_);
      }
    }
  }
}

void HermiteSpline::setNode(size_t node_idx, const std::vector<bool>& optimization_variable_flags, const std::vector<double>& node_values)
{
  size_t input_vector_offset = nr_of_variables_per_node_ * node_idx;

  for (size_t i = 0; i < nr_of_variables_per_node_; i++)
  {
    spline_input_variables_(input_vector_offset + i) = node_values[i];

    if (optimization_variable_flags[i])
    {
      opt_var_to_input_variable_idx_.push_back(input_vector_offset + i);
    }
  }
}

void HermiteSpline::finalizePreparation()
{
  initOptVars();
  setupJacobianSparsityPattern();
}

size_t HermiteSpline::getNumberOfOptVars() { return opt_var_to_input_variable_idx_.size(); }

size_t HermiteSpline::getNumberOfSampleValues() { return nr_of_samples_ * nr_of_dim_ * 2; }

void HermiteSpline::initOptVars()
{
  size_t nr_of_spline_opt_vars = getNumberOfOptVars();

  for (size_t opt_var_idx = 0; opt_var_idx < nr_of_spline_opt_vars; opt_var_idx++)
  {
    size_t input_variable_idx = opt_var_to_input_variable_idx_[opt_var_idx];
    nlp_data_ptr_->optimization_variables_(opt_var_start_idx_ + opt_var_idx) = spline_input_variables_(input_variable_idx);
  }
}

void HermiteSpline::copyOptVars()
{
  size_t nr_of_spline_opt_vars = getNumberOfOptVars();

  for (size_t i = 0; i < nr_of_spline_opt_vars; i++)
  {
    spline_input_variables_(opt_var_to_input_variable_idx_[i]) = nlp_data_ptr_->optimization_variables_[opt_var_start_idx_ + i];
  }
}

void HermiteSpline::findPolynom(double time)
{
  // if given time point lies earlier as the starting point
  // of the last found polynom, begin the search at the beginning
  if (time < current_polynomial_start_time_)
  {
    current_polynomial_start_time_ = 0;
    // last node value is the duration of the polynom
    current_polynomial_end_time_ = spline_input_variables_(nr_of_variables_per_node_ - 1);
    current_node_idx_ = 0;
  }

  while (time > current_polynomial_end_time_ && current_node_idx_ < nr_of_spline_nodes_)
  {
    current_node_idx_++;
    current_polynomial_start_time_ = current_polynomial_end_time_;
    current_polynomial_end_time_ += spline_input_variables_(current_node_idx_ * nr_of_variables_per_node_ + nr_of_variables_per_node_ - 1);
  }
}

void HermiteSpline::setupJacobianSparsityPattern()
{
  size_t first_relevant_opt_var_idx = 0;
  for (size_t i = 0; i < nr_of_samples_; i++)
  {
    double sample_time = i * sample_delta_time_;

    findPolynom(sample_time);

    // scan to the first opt var that is relevant for the polynom that is determining the values of the current sample
    while (first_relevant_opt_var_idx < opt_var_to_input_variable_idx_.size() &&
           opt_var_to_input_variable_idx_[first_relevant_opt_var_idx] < current_node_idx_ * nr_of_variables_per_node_)
    {
      first_relevant_opt_var_idx++;
    }

    // determine the first row in the calculation cache jacobian that is representing the current sample
    const size_t sample_row_start_idx = calculation_cache_start_idx_ + i * (nr_of_variables_per_node_ - 1);

    // iterate over all rows of the current sample in the jacobian that are nr_of_dim_ for the position and nr_of_dim_ for the velocity
    for (size_t row_offset = 0; row_offset < 2 * nr_of_dim_; row_offset++)
    {
      // iterate over all optimization variables that are relevant for the current sample
      for (size_t opt_var_idx = first_relevant_opt_var_idx;
           // check for still having more optimization variables
           opt_var_idx < opt_var_to_input_variable_idx_.size() &&
           // check whether the optimization variable still influences the current polynomial or just the next one
           opt_var_to_input_variable_idx_[opt_var_idx] < current_node_idx_ * nr_of_variables_per_node_ + nr_of_variables_start_and_end_node_;
           opt_var_idx++)
      {
        // this is true if the optimization variable represents the duration of the polynomial and thus influences all values of the sample for position and velocity
        if (opt_var_idx % nr_of_variables_per_node_ == (nr_of_variables_per_node_ - 1))
        {
          nlp_data_ptr_->calculation_cache_.getInternalJacobian().insert(sample_row_start_idx + row_offset, opt_var_start_idx_ + opt_var_idx) = 0;
        }
        else
        {
          // this checks whether the position of the sample along the dimension that is represented by the current row is influenced by the optimization variable
          if (row_offset == (opt_var_to_input_variable_idx_[opt_var_idx] % nr_of_variables_per_node_) % nr_of_dim_ ||
              // this checks whether the velocity of the sample along the dimension that is represented by the current row is influenced by the optimization variable
              row_offset == nr_of_dim_ + (opt_var_to_input_variable_idx_[opt_var_idx] % nr_of_variables_per_node_) % nr_of_dim_)
          {
            nlp_data_ptr_->calculation_cache_.getInternalJacobian().insert(sample_row_start_idx + row_offset, opt_var_start_idx_ + opt_var_idx) = 0;
          }
        }
      }
    }
  }

  // make the changes of the structure of the interal jaobian visible to the map
  nlp_data_ptr_->calculation_cache_.applyInternalStructureChangesToMap();
}

void HermiteSpline::updatePolynomialJacobian(size_t start_node_idx, double local_time)
{
  size_t input_vector_offset = start_node_idx * nr_of_variables_per_node_;
  jac_input_temp_.segment(0, nr_of_variables_start_and_end_node_) = spline_input_variables_.segment(input_vector_offset, nr_of_variables_start_and_end_node_);
  jac_input_temp_(nr_of_variables_start_and_end_node_) = local_time;

  double a, b, c, d, T, T2, T3, T4, t, t2, t3;
  T = jac_input_temp_(2 * nr_of_dim_);
  T2 = T * T;
  T3 = T2 * T;
  T4 = T3 * T;

  t = jac_input_temp_(4 * nr_of_dim_ + 1);
  t2 = t * t;
  t3 = t2 * t;

  for (size_t i = 0; i < nr_of_dim_; i++)
  {
    a = jac_input_temp_(i);
    b = jac_input_temp_(nr_of_dim_ + i);
    c = jac_input_temp_(2 * nr_of_dim_ + 1 + i);
    d = jac_input_temp_(3 * nr_of_dim_ + 1 + i);

    // calculate all derivatives of the position
    polynomial_jacobian_(i, i) = 2 * t3 / T3 - 3 * t2 / T2 + 1;                                                                                                       // ds/da
    polynomial_jacobian_(i, nr_of_dim_ + i) = t3 / T2 - 2 * t2 / T + t;                                                                                               // ds/db
    polynomial_jacobian_(i, 2 * nr_of_dim_) = t2 * (6 * a / T3 + 2 * b / T2 - 6 * c / T3 + d / T2) + t3 * (-6 * a / T4 - 2 * b / T3 + 6 * c / T4 - 2 * d / T3);       // ds/dT
    polynomial_jacobian_(i, 2 * nr_of_dim_ + 1 + i) = 3 * t2 / T2 - 2 * t3 / T3;                                                                                      // ds/dc
    polynomial_jacobian_(i, 3 * nr_of_dim_ + 1 + i) = t3 / T2 - t2 / T;                                                                                               // ds/dd
    polynomial_jacobian_(i, 4 * nr_of_dim_ + 1) = b + 2 * t * (-3 * a / T2 - 2 * b / T + 3 * c / T2 - d / T) + 3 * t2 * (2 * a / T3 + b / T2 - 2 * c / T3 + d / T2);  // ds/dt

    // calculate all derivatives of the velocity
    polynomial_jacobian_(nr_of_dim_ + i, i) = 6 * t2 / T3 - 6 * t / T2;                  // dv/da
    polynomial_jacobian_(nr_of_dim_ + i, nr_of_dim_ + i) = 3 * t2 / T2 - 4 * t / T + 1;  // dv/db
    polynomial_jacobian_(nr_of_dim_ + i, 2 * nr_of_dim_) =
        2 * t * (6 * a / T3 + 2 * b / T2 - 6 * c / T3 + d / T2) + 3 * t2 * (-6 * a / T4 - 2 * b / T3 + 6 * c / T4 - 2 * d / T3);                                          // dv/dT
    polynomial_jacobian_(nr_of_dim_ + i, 2 * nr_of_dim_ + 1 + i) = 6 * t / T2 - 6 * t2 / T3;                                                                              // dv/dc
    polynomial_jacobian_(nr_of_dim_ + i, 3 * nr_of_dim_ + 1 + i) = 3 * t2 / T2 - 2 * t / T;                                                                               // dv/dd
    polynomial_jacobian_(nr_of_dim_ + i, 4 * nr_of_dim_ + 1) = 2 * (-3 * a / T2 - 2 * b / T + 3 * c / T2 - d / T) + 6 * t * (2 * a / T3 + b / T2 - 2 * c / T3 + d / T2);  // dv/dt
  }
}

}  // namespace optiminf
