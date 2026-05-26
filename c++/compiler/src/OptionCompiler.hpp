#pragma once

#include <iostream>
#include <string>

#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"

namespace d4 {

class OptionCompiler : public OptionGroup {
 public:
  OptionCompiler(const std::string& name = "compiler",
                 const std::string& description = "Counter specific options")
      : OptionGroup(name, description) {}

  Option<std::string> dumpFile{"dump-file", "Output format", "/dev/null"};
  Option<std::string> queryFile{"query-file", "Output style", "/dev/null"};

  std::vector<OptionBase*> getAllOptions() override {
    return {&dumpFile, &queryFile};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionCompiler& dt) {
    out << " Option Compiler: dump-file(" << dt.dumpFile.get()
        << ") query-file(" << dt.queryFile.get() << ")\n";
    return out;
  }
};

}  // namespace d4