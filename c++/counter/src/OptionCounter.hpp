#pragma once

#include <iostream>
#include <string>

#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"

namespace d4 {

class OptionCounter : public OptionGroup {
 public:
  OptionCounter(const std::string& name = "counter",
                const std::string& description = "Counter specific options")
      : OptionGroup(name, description) {}

  Option<std::string> informat{"input-format", "Input format", "cnf"};
  Option<std::string> format{"format", "Output format", "s"};
  Option<std::string> outFormat{"outFormat", "Output style", "classic"};

  std::vector<OptionBase*> getAllOptions() override {
    return {&informat, &format, &outFormat};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionCounter& dt) {
    out << " Option Counter: input-format(" << dt.informat.get() << " format("
        << dt.format.get() << ") outFormat(" << dt.outFormat.get() << ")\n";
    return out;
  }
};

}  // namespace d4
