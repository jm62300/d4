#pragma once

#include <iostream>
#include <string>

#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"

namespace d4 {

/**
 * @brief Group of options specific to the compiler demo.
 *
 * This class handles command-line options for dumping the compiled
 * decision-DNNF to a file and for performing queries against it.
 */
class OptionCompiler : public OptionGroup {
 public:
  /**
   * @brief Construct a new OptionCompiler group.
   *
   * @param name The name of the option group (defaults to "compiler").
   * @param description A brief description of the group's purpose.
   */
  OptionCompiler(const std::string& name = "compiler",
                 const std::string& description = "Counter specific options")
      : OptionGroup(name, description) {}

  /**
   * @brief Path to the file where the compiled decision-DNNF should be dumped.
   * Defaults to "/dev/null" (no dump).
   */
  Option<std::string> dumpFile{"dump-file", "Output format", "/dev/null"};

  /**
   * @brief Path to a file containing queries to be executed on the compiled
   * form. Defaults to "/dev/null" (no queries).
   */
  Option<std::string> queryFile{"query-file", "Output style", "/dev/null"};

  /**
   * @brief Returns a list of all options contained in this group.
   * @return A vector of pointers to the internal options.
   */
  std::vector<OptionBase*> getAllOptions() override {
    return {&dumpFile, &queryFile};
  }

  /** @brief Overload for printing the state of the compiler options. */
  friend std::ostream& operator<<(std::ostream& out, const OptionCompiler& dt) {
    out << " Option Compiler: dump-file(" << dt.dumpFile.get()
        << ") query-file(" << dt.queryFile.get() << ")\n";
    return out;
  }
};

}  // namespace d4