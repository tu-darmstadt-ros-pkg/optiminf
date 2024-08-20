#pragma once

#include "optiminf/data_structs/nlp_data.h"
#include "optiminf/data_structs/manager_container.h"

namespace optiminf
{
class ManagerContainer;

class ExporterBase
{
public:
  using Ptr =std::shared_ptr<ExporterBase>;

  ExporterBase() = default;

  virtual ~ExporterBase() = default;

  /**
   * @brief setupExporter gets called by the NlpManager during the addExporter function in order to provide the class with the pointer to manager container and the nlp data.
   * @details There is no need to call this function in custom code.
   * @param manager_container_ptr
   * @param nlp_data_ptr
   */
  void setupExporter(std::shared_ptr<const ManagerContainer> manager_container_ptr, NlpData::ConstPtr nlp_data_ptr);

  /**
   * @brief exportSolution should be implemented by the child class and is called during NlpManager::exportSolution()
   */
  virtual void exportSolution() = 0;

protected:
  /**
   * @brief getManagerContainerPtr read accessor to the manager container for retrieving the solution
   * @return read accessor to the manager container
   */
  const std::shared_ptr<const ManagerContainer>& getManagerContainerPtr();

  /**
   * @brief getNlpDataPtr read accessor to the nlp data struct for retrieving the solution
   * @return read accessor to the nlp data struct
   */
  const NlpData::Ptr& getNlpDataPtr();

private:
 std::shared_ptr<const ManagerContainer> manager_container_ptr_;
  NlpData::ConstPtr nlp_data_ptr_;
};
}  // namespace optiminf
