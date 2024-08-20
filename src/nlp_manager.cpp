#include "optiminf/nlp_manager/nlp_manager.h"

#include "paraminf/parameter_interface.h"
#include "paraminf/yaml_io_handler.h"

#include "optiminf/error_handling/optiminf_exception.h"
#include "optiminf/util/parameter_naming_util.h"

namespace optiminf
{
NlpManager::NlpManager()
{
  manager_container_ptr_ = std::make_shared<ManagerContainer>();
  nlp_data_ptr_ = std::make_shared<NlpData>();
}

void NlpManager::setupNlp(const SetupManagerBase::Ptr setup_manager_ptr, const std::string& config_file_path) { setupNlp(setup_manager_ptr, parseConfigFile(config_file_path)); }

void NlpManager::setupNlp(SetupManagerBase::Ptr setup_manager_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr)
{
  manager_container_ptr_->parameter_interface_ptr_ = parameter_interface_ptr;
  setup_manager_ptr->setupNlp(manager_container_ptr_, nlp_data_ptr_);

  manager_container_ptr_->parameter_interface_ptr_->getParam(ParameterNamingUtil::CONSTRAINT_DEBUG_LEVEL_PARAM_NAME, constraint_debug_level_);
  manager_container_ptr_->parameter_interface_ptr_->getParam(ParameterNamingUtil::COST_DEBUG_LEVEL_PARAM_NAME, cost_debug_level_);
}

void NlpManager::prepareNlp() { manager_container_ptr_->preparation_manager_ptr_->prepareNlp(); }

SolverResultStatus NlpManager::solveNlp()
{
  nlp_data_ptr_->is_optimization_running_ = true;
  SolverResultStatus result_status = manager_container_ptr_->solver_interface_ptr_->runSolver();
  nlp_data_ptr_->is_optimization_running_ = false;

  if (constraint_debug_level_ >= 1 || cost_debug_level_ >= 1)
    std::cout << "----------------------Solver finished-----------------------------------\n"
              << "Result status : " << solverResultStatusToString(result_status) << std::endl;
  if (cost_debug_level_ >= 1)
    manager_container_ptr_->cost_manager_ptr_->printCostValues();
  if (constraint_debug_level_ >= 1)
    manager_container_ptr_->constraint_manager_ptr_->printNrOfConstraintViolations();
  if (constraint_debug_level_ >= 2)
    manager_container_ptr_->constraint_manager_ptr_->printDebugInfo();

  return result_status;
}

void NlpManager::exportSolution()
{
  for (const auto& exporter_ptr : manager_container_ptr_->exporter_ptr_vec_)
  {
    exporter_ptr->exportSolution();
  }
}

void NlpManager::addExporter(const ExporterBase::Ptr exporter_ptr)
{
  exporter_ptr->setupExporter(manager_container_ptr_, nlp_data_ptr_);
  manager_container_ptr_->exporter_ptr_vec_.push_back(exporter_ptr);
}

paraminf::ParameterInterface::Ptr NlpManager::parseConfigFile(const std::string& config_file_path)
{
  // parse yaml file
  auto param_interface_ptr = std::make_shared<paraminf::ParameterInterface>();
  if (!paraminf::YamlIOHandler::readAndAddParametersFromFile(config_file_path, *param_interface_ptr))
  {
    throw ParsingError("Config file could not be read, path: " + config_file_path);
  }

  return param_interface_ptr;
}

}  // namespace optiminf
