#pragma once

#include <string>

namespace d4::api {

/**
 * @brief Class representing the result of a model counting execution.
 * It holds the model count result and query interfaces for solver statistics.
 */
class CounterResult {
 public:
  virtual ~CounterResult() = default;

  /**
   * @brief Returns the model count result as a string.
   */
  virtual std::string getResult() const = 0;

  /**
   * @brief Returns the number of recursive calls made during solving.
   */
  virtual unsigned getNbRecursiveCalls() const = 0;

  /**
   * @brief Returns the number of formula splits during solving.
   */
  virtual unsigned getNbSplits() const = 0;

  /**
   * @brief Returns the number of decision nodes created during solving.
   */
  virtual unsigned getNbDecisionNodes() const = 0;

  /**
   * @brief Returns the solve time in seconds.
   */
  virtual float getSolveTime() const = 0;
};

} // namespace d4::api
