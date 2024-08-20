#include "optiminf/error_handling/optiminf_exception.h"

namespace optiminf
{
OptiminfException::OptiminfException(const std::string& msg) { msg_ = msg; }

OptiminfException::OptiminfException(const std::string&& msg) { msg_ = std::move(msg); }

const char* OptiminfException::what() const noexcept { return msg_.c_str(); }

ParsingError::ParsingError(const std::string& msg)
  : OptiminfException(msg)
{}

ParsingError::ParsingError(const std::string&& msg)
  : OptiminfException(std::move(msg))
{}

SetupError::SetupError(const std::string& msg)
  : OptiminfException(msg)
{}

SetupError::SetupError(const std::string&& msg)
  : OptiminfException(std::move(msg))
{}

PreparationError::PreparationError(const std::string& msg)
  : OptiminfException(msg)
{}

PreparationError::PreparationError(const std::string&& msg)
  : OptiminfException(std::move(msg))
{}

LogicError::LogicError(const std::string& msg)
  : OptiminfException(msg)
{}

LogicError::LogicError(const std::string&& msg)
  : OptiminfException(std::move(msg))
{}

SolvingError::SolvingError(const std::string& msg)
  : OptiminfException(msg)
{}

SolvingError::SolvingError(const std::string&& msg)
  : OptiminfException(std::move(msg))
{}

NumericError::NumericError(const std::string& msg)
  : OptiminfException(msg)
{}

NumericError::NumericError(const std::string&& msg)
  : OptiminfException(std::move(msg))
{}

InvalidArgument::InvalidArgument(const std::string& msg)
  : OptiminfException(msg)
{}

InvalidArgument::InvalidArgument(const std::string&& msg)
  : OptiminfException(std::move(msg))
{}

}  // namespace optiminf
