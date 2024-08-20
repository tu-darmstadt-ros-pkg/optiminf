#include <gtest/gtest.h>

#include "optiminf/nlp_manager/nlp_manager.h"

#include "optiminf/test/unit_test_setup_manager.h"
#include "optiminf/test/simple_exporter.h"

namespace optiminf
{
namespace test
{
TEST(FullSetupTest, SolveNlpTest)
{
  const double result_tolerance = 0.01;
  NlpManager nlp_manager;

  SimpleExporter::Ptr exporter_ptr = std::make_shared<SimpleExporter>();
  nlp_manager.addExporter(exporter_ptr);

  std::string config_path = SOURCE_DIR "/test/test_config/test_config.yaml";

  bool use_cost = true;
  bool use_constraint = true;

  auto setup_manager_ptr = std::make_shared<UnitTestSetupManager>(use_cost, use_constraint);
  nlp_manager.setupNlp(setup_manager_ptr, config_path);
  // ASSERT_NO_THROW(nlp_manager.setupNlp(setup_manager_ptr, config_path));
  ASSERT_NO_THROW(nlp_manager.prepareNlp());
  ASSERT_NO_THROW(nlp_manager.solveNlp());
  ASSERT_NO_THROW(nlp_manager.exportSolution());

  /* check expected solution */
  const Eigen::VectorXd solution = exporter_ptr->getSolution();

  const std::vector<double> expected_solution = { 1, 2, 10 - sqrt(3) };

  EXPECT_NEAR(solution(0), expected_solution[0], result_tolerance);
  EXPECT_NEAR(solution(1), expected_solution[1], result_tolerance);
  EXPECT_NEAR(solution(2), expected_solution[2], result_tolerance);

  /* check for correctly calculating the cost value */
  const double expected_cost = 2 * (expected_solution[0] + expected_solution[1] * expected_solution[1] + expected_solution[2]);

  EXPECT_NEAR(setup_manager_ptr->getNlpDataPtrForTest()->total_cost_, expected_cost, result_tolerance);

  /* check for correctly calculating the constraint values */
  const Eigen::VectorXd constraint_values = setup_manager_ptr->getNlpDataPtrForTest()->constraint_values_;

  const std::vector<double> expected_constraint_vaules = { 1, 2, 10 - sqrt(3), 10 };

  EXPECT_NEAR(constraint_values(0), expected_constraint_vaules[0], result_tolerance);
  EXPECT_NEAR(constraint_values(1), expected_constraint_vaules[1], result_tolerance);
  EXPECT_NEAR(constraint_values(2), expected_constraint_vaules[2], result_tolerance);
  EXPECT_NEAR(constraint_values(3), expected_constraint_vaules[3], result_tolerance);
}

TEST(FullSetupTest, ConfigureTest)
{
  NlpManager nlp_manager;

  SimpleExporter::Ptr exporter_ptr = std::make_shared<SimpleExporter>();
  nlp_manager.addExporter(exporter_ptr);

  std::string config_path = SOURCE_DIR "/test/test_config/test_config.yaml";

  bool use_cost = true;
  bool use_constraint = true;

  auto initialization_manager_ptr = std::make_shared<UnitTestSetupManager>(use_cost, use_constraint);

  ASSERT_NO_THROW(nlp_manager.setupNlp(initialization_manager_ptr, config_path));
  ASSERT_NO_THROW(nlp_manager.prepareNlp());

  NlpData::Ptr nlp_data_ptr = initialization_manager_ptr->getNlpDataPtrForTest();

  const size_t nr_of_opt_variables = 3;
  EXPECT_EQ(nlp_data_ptr->optimization_variables_.size(), nr_of_opt_variables);
  EXPECT_EQ(nlp_data_ptr->calculation_cache_.jacobian_map_.cols(), nr_of_opt_variables);
  EXPECT_EQ(nlp_data_ptr->constraint_jacobian_.cols(), nr_of_opt_variables);
  EXPECT_EQ(nlp_data_ptr->cost_gradient_.cols(), nr_of_opt_variables);

  const size_t nr_of_cc_entries = 7;
  EXPECT_EQ(nlp_data_ptr->calculation_cache_.values_.size(), nr_of_cc_entries);
  EXPECT_EQ(nlp_data_ptr->calculation_cache_.jacobian_map_.rows(), nr_of_cc_entries);

  const size_t nr_of_constraints = 4;
  EXPECT_EQ(nlp_data_ptr->lower_constraint_bounds_.size(), nr_of_constraints);
  EXPECT_EQ(nlp_data_ptr->upper_constraint_bounds_.size(), nr_of_constraints);
  EXPECT_EQ(nlp_data_ptr->constraint_jacobian_.rows(), nr_of_constraints);
}
}  // namespace test
}  // namespace optiminf
