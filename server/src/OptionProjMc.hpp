#pragma once

#include <optree/OptionGroup.hpp>
#include <optree/Option.hpp>
#include <vector>
#include <string>

namespace d4 {

class OptionProjMc : public optree::OptionGroup {
 public:
  OptionProjMc(const std::string& name = "projMC",
               const std::string& description = "Projected Model Counting options")
      : OptionGroup(name, description) {}

  optree::Option<bool> refinement{"refinement", "Refinement activated or not", true};

  std::vector<optree::OptionBase*> getAllOptions() override {
    return {&refinement};
  }
};

}  // namespace d4
