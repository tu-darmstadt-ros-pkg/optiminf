#pragma once

#include "optiminf/export/exporter_base.h"

namespace optiminf
{
namespace test
{
class SimpleExporter : public optiminf::ExporterBase
{
public:
  using Ptr =std::shared_ptr<SimpleExporter>;

  SimpleExporter() = default;

  /**
   * @brief exportSolution stores the solution vector s.t. it can be retrieved by calling getSolution()
   */
  void exportSolution() final;

  /**
   * @brief getSolution returns the last exported solution
   * @return solution vector
   */
  const Eigen::VectorXd& getSolution();

private:
  Eigen::VectorXd solution_;
};
}  // namespace test
}  // namespace optiminf
