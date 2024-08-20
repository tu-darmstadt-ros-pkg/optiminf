#pragma once

#include <string>

namespace optiminf
{
/**
 * @brief The ParameterNamingUtil class contains definitions for the naming of parameters that can be set in the config file.
 */
class ParameterNamingUtil
{
public:
  static const std::string ACCEPTABLE_CONSTRAINT_VIOLATION_ON_SOLVER_FAILURE_PARAM_NAME;
  static const std::string ACCEPTABLE_SOLVER_FAILURES_PARAM_NAME;
  static const std::string CHECK_FOR_NON_FINITE_VALUES_PARAM_NAME;
  static const std::string COST_DEBUG_LEVEL_PARAM_NAME;
  static const std::string CONSTRAINT_DEBUG_LEVEL_PARAM_NAME;
};
}  // namespace optiminf
