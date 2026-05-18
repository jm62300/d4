#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

#include "src/options/EnumMetadata.hpp"
#include "src/options/OptionBase.hpp"
#include "src/options/OptionRegistry.hpp"

namespace d4 {

/**
 * @brief Concrete implementation of an option for a specific type T.
 */
template <typename T>
class Option : public OptionBase {
 public:
  Option() : m_name(""), m_description(""), m_value(T()) {}

  Option(const std::string& name, const std::string& description,
         T defaultValue)
      : m_name(name), m_description(description), m_value(defaultValue) {}

  std::string getName() const override { return m_name; }
  std::string getDescription() const override { return m_description; }

  std::string getValueAsString() const override {
    if constexpr (std::is_same_v<T, std::string>) {
      return m_value;
    } else if constexpr (std::is_same_v<T, bool>) {
      return m_value ? "true" : "false";
    } else if constexpr (std::is_enum_v<T>) {
      return enum_to_string(m_value);
    } else {
      std::stringstream ss;
      ss << m_value;
      return ss.str();
    }
  }

  void setFromString(const std::string& value) override {
    if constexpr (std::is_same_v<T, std::string>) {
      m_value = value;
    } else if constexpr (std::is_same_v<T, bool>) {
      m_value =
          (value == "true" || value == "1" || value == "yes" || value == "on");
    } else if constexpr (std::is_enum_v<T>) {
      try {
        m_value = resolve_enum<T>(value);
      } catch (...) {
        // Enums need specific managers for string conversion, handled
        // externally
      }
    } else {
      std::stringstream ss(value);
      ss >> m_value;
    }
  }

  std::string getTypeName() const override {
    if constexpr (std::is_same_v<T, std::string>) return "string";
    if constexpr (std::is_same_v<T, bool>) return "bool";
    if constexpr (std::is_same_v<T, int>) return "int";
    if constexpr (std::is_same_v<T, unsigned>) return "uint";
    if constexpr (std::is_same_v<T, float>) return "float";
    if constexpr (std::is_same_v<T, double>) return "double";
    if constexpr (std::is_enum_v<T>) return "enum";
    return "unknown";
  }

  std::string getPossibleValues() const override {
    if constexpr (std::is_enum_v<T>) {
      return get_enum_doc<T>();
    }
    return "";
  }

  const T& get() const { return m_value; }
  void set(const T& value) { m_value = value; }

  /** @brief Implicit conversion to T. */
  operator T() const { return m_value; }

  /** @brief Assignment from T. */
  Option& operator=(const T& value) {
    m_value = value;
    return *this;
  }

  void registerTo(OptionRegistry& registry,
                  const std::string& prefix = "") override {
    registry.registerOption(prefix + getName(), this);
  }

  void setPropagation(const std::string& name, const std::string& value) override {
    if (m_name == name) {
      setFromString(value);
    }
  }

 private:
  std::string m_name;
  std::string m_description;
  T m_value;
};

}  // namespace d4
