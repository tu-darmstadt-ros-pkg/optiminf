#include "optiminf/nlp_manager/nlp_manager.h"

#include "optiminf/test/unit_test_setup_manager.h"
#include "optiminf/test/simple_exporter.h"

int main()
{
  optiminf::NlpManager nlp_manager;

  optiminf::test::SimpleExporter::Ptr exporter_ptr = std::make_shared<optiminf::test::SimpleExporter>();
  nlp_manager.addExporter(exporter_ptr);

  std::string config_path = SOURCE_DIR "/example/config/demo_config.yaml";

  bool use_cost = true;
  bool use_constraint = true;

  auto setup_manager_ptr = std::make_shared<optiminf::test::UnitTestSetupManager>(use_cost, use_constraint);
  nlp_manager.setupNlp(setup_manager_ptr, config_path);
  nlp_manager.prepareNlp();
  nlp_manager.solveNlp();
  nlp_manager.exportSolution();

  std::cout << "Solution Vector: \n" << exporter_ptr->getSolution() << std::endl;
  return 0;
}
