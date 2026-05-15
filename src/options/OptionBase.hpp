#pragma once
#include <string>

namespace d4 {
class OptionRegistry;

/**
 * @brief Virtual base class for all options.
 */
class OptionBase {
 public:
  virtual ~OptionBase() = default;

  /** @brief Get the name of the option. */
  virtual std::string getName() const = 0;

  /** @brief Get the description of the option. */
  virtual std::string getDescription() const = 0;

  /** @brief Get the current value as a string. */
  virtual std::string getValueAsString() const = 0;

  /** @brief Set the value from a string. */
  virtual void setFromString(const std::string& value) = 0;

  /** @brief Get the type name of the option. */
  virtual std::string getTypeName() const = 0;

  /** @brief Get a list of possible values (for enums). */
  virtual std::string getPossibleValues() const = 0;

  /** @brief Register the option to a registry with a prefix. */
  virtual void registerTo(OptionRegistry& registry, const std::string& prefix = "") = 0;

  /** @brief Try to set an option by name within this object or its children. */
  virtual void setPropagation(const std::string& name, const std::string& value) = 0;
};

}  // namespace d4
