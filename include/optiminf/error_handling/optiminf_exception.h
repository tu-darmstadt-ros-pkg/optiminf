#pragma once

#include <exception>
#include <string>

namespace optiminf
{
class OptiminfException : public std::exception
{
public:
  OptiminfException(const std::string& msg);
  OptiminfException(const std::string&& msg);

  virtual const char* what() const noexcept;

private:
  std::string msg_;
};

/**
 * @brief The ParsingError class represents an error that occured during the parsing of a config file
 */
class ParsingError : public OptiminfException
{
public:
  ParsingError(const std::string& msg);

  ParsingError(const std::string&& msg);
};

/**
 * @brief The SetupError class represents an error during the setup phase of the NLP
 */
class SetupError : public OptiminfException
{
public:
  SetupError(const std::string& msg);

  SetupError(const std::string&& msg);
};

/**
 * @brief The PreparationError class represents an error error during the preparation phase of the NLP
 */
class PreparationError : public OptiminfException
{
public:
  PreparationError(const std::string& msg);

  PreparationError(const std::string&& msg);
};

/**
 * @brief The InvalidArgument class represents that at least one input argument of a function is invalid.
 */
class InvalidArgument : public OptiminfException
{
public:
  InvalidArgument(const std::string& msg);

  InvalidArgument(const std::string&& msg);
};

/**
 * @brief The NumericError class represents an error regarding numeric value, e.g. because a value is non-finite.
 */
class NumericError : public OptiminfException
{
public:
  NumericError(const std::string& msg);

  NumericError(const std::string&& msg);
};

/**
 * @brief The LogicError class represents an error regarding the program logic, e.g. because the order of the calls to some API where in the wrong order.
 */
class LogicError : public OptiminfException
{
public:
  LogicError(const std::string& msg);

  LogicError(const std::string&& msg);
};

/**
 * @brief The SolvingError class represents an error that occured while solving the optimization problem
 */
class SolvingError : public OptiminfException
{
public:
  SolvingError(const std::string& msg);

  SolvingError(const std::string&& msg);
};

}  // namespace optiminf
