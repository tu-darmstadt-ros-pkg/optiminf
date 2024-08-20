#pragma once

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/manager_container.h"
#include "optiminf/setup/setup_manager_base.h"

namespace optiminf
{
class NlpManager
{
public:
  using Ptr =std::shared_ptr<NlpManager>;

  NlpManager();

  /**
   * @brief setupNlp performs all operations that have to be done just once to setup the NLP.
   * @details The setup manager and the config file determine the details how the problem is build. This function has to be called prior to prepareNlp() or solveNlp().
   * This function can take a considerable amount of time depending on the complexity of the implementation and should be called outside any real time critical path
   * @param setup_manager_ptr pointer to child class of SetupManagerBase that implements the operations that need to be done to setup the nlp
   * @param config_file_path path to yaml config file that contains parameters like solver configuration, weights, etc.
   */
  void setupNlp(SetupManagerBase::Ptr setup_manager_ptr, const std::string& config_file_path);

  /**
   * @brief setupNlp performs all operations that have to be done just once to setup the NLP.
   * @details The setup manager and the config file determine the details how the problem is build. This function has to be called prior to prepareNlp() or solveNlp().
   * This function can take a considerable amount of time depending on the complexity of the implementation and should be called outside any real time critical path
   * @param setup_manager_ptr pointer to child class of SetupManagerBase that implements the operations that need to be done to setup the nlp
   * @param parameter_interface_ptr pointer to the parameter interface that contains parameters like solver configuration, weights, etc.
   * @details always use this overload in case you want that changes to parameters after the call to this function are visible to the managers, constraints and costs
   */
  void setupNlp(SetupManagerBase::Ptr setup_manager_ptr, paraminf::ParameterInterface::Ptr parameter_interface_ptr);

  /**
   * @brief prepareNlp performs all operations that are specific to the concrete values and need to be done just once before starting the solver, like initializing the optimization
   * variables or allocating memory that depends on changing input parameters
   */
  void prepareNlp();

  /**
   * @brief solveNlp starts the solver and blocks until the solver has finished or was aborted
   * @return returns the result status of the solving process
   */
  SolverResultStatus solveNlp();

  /**
   * @brief exportSolution exports the solution calling all exporters that have been added by calling addExporter() in the same order as they have been added
   */
  void exportSolution();

  /**
   * @brief addExporter adds an exporter that gets called by exportSolution()
   * @param exporter_ptr pointer to child class of ExporterBase
   */
  void addExporter(ExporterBase::Ptr exporter_ptr);

private:
  paraminf::ParameterInterface::Ptr parseConfigFile(const std::string& config_file_path);
  NlpData::Ptr nlp_data_ptr_;
  ManagerContainer::Ptr manager_container_ptr_;

  int constraint_debug_level_ = 0;
  int cost_debug_level_ = 0;
};
}  // namespace optiminf
