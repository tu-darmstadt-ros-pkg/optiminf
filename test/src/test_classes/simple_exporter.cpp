#include "optiminf/test/simple_exporter.h"

namespace optiminf
{
namespace test
{
void SimpleExporter::exportSolution() { solution_ = getNlpDataPtr()->optimization_variables_; }

const Eigen::VectorXd& SimpleExporter::getSolution() { return solution_; }
}  // namespace test
}  // namespace optiminf
