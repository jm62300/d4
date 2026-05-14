#pragma once
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "src/options/EnumMetadata.hpp"
#include "src/options/OptionBase.hpp"

namespace d4 {

/**
 * @brief Registry to manage dot-notation paths for options.
 */
class OptionRegistry {
 public:
  /** @brief Register an option with a specific path. */
  void registerOption(const std::string& path, OptionBase* option) {
    m_options[path] = option;
  }

  /** @brief Check if an option exists. */
  bool hasOption(const std::string& path) const {
    return m_options.count(path) > 0;
  }

  /** @brief Set an option from a string value. */
  void setOption(const std::string& path, const std::string& value) {
    auto it = m_options.find(path);
    if (it != m_options.end()) it->second->setFromString(value);
  }  // setOption

  /** @brief Parse command line arguments and update registered options. */
  void parseArgv(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg.size() > 2 && arg.substr(0, 2) == "--") {
        std::string key, value;
        auto eqPos = arg.find('=');
        if (eqPos != std::string::npos) {
          key = arg.substr(2, eqPos - 2);
          value = arg.substr(eqPos + 1);
        } else if (i + 1 < argc) {
          key = arg.substr(2);
          std::string next = argv[i + 1];
          if (next.size() < 2 || next.substr(0, 2) != "--") {
            value = next;
            ++i;
          } else {
            value = "true";
          }
        } else {
          key = arg.substr(2);
          value = "true";
        }

        setOption(key, value);
      }
    }
  }  // parseArgv

  /** @brief Display a pretty tree of all registered options. */
  void displayHelp(std::ostream& out) const {
    out << "\n\033[1;36mAvailable Options (use "
           "--path.to.option=value):\033[0m\n";
    std::set<std::string> displayed_nodes;
    renderTree(out, "", "", displayed_nodes);
    out << std::endl;
  }

  void renderTree(std::ostream& out, const std::string& prefix,
                  const std::string& currentPath,
                  std::set<std::string>& displayed) const {
    // Collect immediate children (next segment in dot notation)
    std::map<std::string, bool> children;  // name -> isLeaf
    for (const auto& [path, opt] : m_options) {
      if (currentPath.empty() ||
          (path.size() > currentPath.size() &&
           path.substr(0, currentPath.size()) == currentPath)) {
        std::string relative =
            currentPath.empty() ? path : path.substr(currentPath.size() + 1);
        auto dotPos = relative.find('.');
        if (dotPos == std::string::npos) {
          children[relative] = true;
        } else {
          children[relative.substr(0, dotPos)] = false;
        }
      }
    }

    size_t count = 0;
    for (auto it = children.begin(); it != children.end(); ++it, ++count) {
      bool isLast = (count == children.size() - 1);
      std::string name = it->first;
      bool isLeaf = it->second;
      std::string fullPath =
          currentPath.empty() ? name : currentPath + "." + name;

      out << prefix << (isLast ? "└── " : "├── ") << "\033[1;34m" << name
          << "\033[0m";

      if (isLeaf) {
        OptionBase* opt = m_options.at(fullPath);
        out << " [" << opt->getTypeName() << "]"
            << " (default: \033[1;32m" << opt->getValueAsString() << "\033[0m)"
            << " : " << opt->getDescription();

        std::string possible = opt->getPossibleValues();
        if (!possible.empty()) {
          out << " \033[1;33m{" << possible << "}\033[0m";
        }
      }

      if (!isLeaf) {
        if (m_groups.count(fullPath)) {
          out << " : " << m_groups.at(fullPath)->getDescription();
        }
      }
      out << "\n";

      if (!isLeaf) {
        renderTree(out, prefix + (isLast ? "    " : "│   "), fullPath,
                   displayed);
      }
    }
  }

  /** @brief Get access to all registered options. */
  const std::map<std::string, OptionBase*>& getOptions() const {
    return m_options;
  }

  /** @brief Register a group for its description. */
  void registerGroup(const std::string& path, OptionBase* group) {
    m_groups[path] = group;
  }

 private:
  std::map<std::string, OptionBase*> m_options;
  std::map<std::string, OptionBase*> m_groups;
};

}  // namespace d4
