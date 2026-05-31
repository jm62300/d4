#pragma once

#include <iostream>
#include <string>

#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"

namespace d4 {

/**
 * @brief Options specific to the cube-counter demo.
 */
class OptionCubeCounter : public OptionGroup {
 public:
  OptionCubeCounter(const std::string& name = "cube-couter",
                    const std::string& description = "Counter specific options")
      : OptionGroup(name, description) {}

  /**
   * @brief Clause selection strategy for cube-and-count.
   *
   * "primal-cut"           — single primal hypergraph bisection; F_easy
   *                          = cut clauses (those spanning both partitions).
   * "iterative-primal-cut" — recursive bisection up to maxDepth levels;
   *                          F_easy = union of cut clauses at every level.
   */
  Option<std::string> selectorStrategy{
      "selectorStrategy",
      "Clause selection strategy for F_easy "
      "(primal-cut|iterative-primal-cut|high-degree)",
      "high-degree"};

  /**
   * @brief Maximum recursion depth for iterative-primal-cut.
   */
  Option<int> maxDepth{
      "maxDepth", "Recursion depth for iterative-primal-cut (default 3)", 3};

  /**
   * @brief Fraction of variables to target for high-degree-variable strategy.
   */
  Option<double> targetRatio{"targetRatio",
                             "Fraction of variables in V_target for "
                             "high-degree-variable (default 0.1)",
                             0.1};

  /**
   * @brief After the selector runs, absorb every remaining clause whose
   *        variables are all already in vars(F_easy). On by default.
   */
  Option<bool> extendEasy{"extendEasy",
                          "Absorb clauses that do not grow vars(F_easy) (default true)",
                          true};

  std::vector<OptionBase*> getAllOptions() override {
    return {&selectorStrategy, &maxDepth, &targetRatio, &extendEasy};
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionCubeCounter& dt) {
    out << " Option CubeCounter: selectorStrategy(" << dt.selectorStrategy.get()
        << ") maxDepth(" << dt.maxDepth.get() << ")\n";
    return out;
  }
};

}  // namespace d4
