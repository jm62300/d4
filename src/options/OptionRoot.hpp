#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "src/methods/MethodManager.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"

namespace d4 {

/**
 * @brief Base class for configuration structures containing common options.
 */
class OptionRoot : public OptionGroup {
 public:
  OptionRoot() : OptionGroup("", "") {}

  Option<int> precision{"precision", "The precision for the float", 15};
  Option<bool> isFloat{"isFloat", "If the count is computed as a float or not",
                       false};
  Option<std::string> inputName{"inputName", "Path to get the input file", ""};
  Option<ProblemInputType> problemInputType{"problemInputType",
                                            "The input type", PB_CNF};

  virtual ~OptionRoot() = default;

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&precision, (OptionBase*)&isFloat,
            (OptionBase*)&inputName, (OptionBase*)&problemInputType};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionRoot& dt) {
    return out;
  }
};

// For backward compatibility if needed, but we should aim to use OptionRoot or
// specific subclasses.
using LegacyOption = OptionRoot;

}  // namespace d4
