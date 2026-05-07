#pragma once

#include <string>
#include <vector>
#include "src/options/OptionBase.hpp"
#include "src/options/OptionRegistry.hpp"

namespace d4 {

/**
 * @brief A group of options that can be registered hierarchically.
 */
class OptionGroup : public OptionBase {
 public:
  OptionGroup(const std::string& name, const std::string& description)
      : m_name(name), m_description(description) {}

  virtual ~OptionGroup() = default;

  /** @brief Get the name of the group. */
  std::string getName() const override { return m_name; }

  /** @brief Get the description of the group. */
  std::string getDescription() const override { return m_description; }

  /** @brief Get the current value as a string (empty for groups). */
  std::string getValueAsString() const override { return ""; }

  /** @brief Set the value from a string (does nothing for groups). */
  void setFromString(const std::string&) override {}

  /** @brief Get the type name of the option. */
  std::string getTypeName() const override { return "group"; }

  /** @brief Get a list of possible values (empty for groups). */
  std::string getPossibleValues() const override { return ""; }

  /** @brief Get all options in this group. */
  virtual std::vector<OptionBase*> getAllOptions() = 0;

  /** @brief Register the group and its children to a registry. */
  void registerTo(OptionRegistry& registry, const std::string& prefix = "") override {
    std::string newPrefix = prefix;
    if (!m_name.empty()) {
      newPrefix += m_name;
      registry.registerGroup(newPrefix, this);
      newPrefix += ".";
    }
    for (auto* opt : getAllOptions()) {
      if (opt) {
        opt->registerTo(registry, newPrefix);
      }
    }
  }

 protected:
  std::string m_name;
  std::string m_description;
};

}  // namespace d4
