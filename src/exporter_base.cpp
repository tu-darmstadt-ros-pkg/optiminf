#include "optiminf/export/exporter_base.h"

namespace optiminf
{
void ExporterBase::setupExporter(const ManagerContainer::ConstPtr manager_container_ptr, const NlpData::Ptr nlp_data_ptr)
{
  manager_container_ptr_ = manager_container_ptr;
  nlp_data_ptr_ = nlp_data_ptr;
}

const std::shared_ptr<const ManagerContainer>& ExporterBase::getManagerContainerPtr() { return manager_container_ptr_; }

const NlpData::Ptr& ExporterBase::getNlpDataPtr() { return nlp_data_ptr_; }
}  // namespace optiminf
