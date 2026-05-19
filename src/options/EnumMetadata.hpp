#pragma once
#include <string>
#include <map>
#include <stdexcept>

namespace d4 {

/**
 * @brief Trait to provide metadata about enums.
 * This is the base template, specialized in EnumRegistry.hpp.
 */
template<typename EnumType>
struct EnumMetadata {
    static std::map<int, std::string> mapping() { return {}; }
    static std::string name() { return "UnknownEnum"; }
};

/**
 * @brief Helper to resolve an enum value from its string label.
 */
template<typename EnumType>
inline EnumType resolve_enum(const std::string& label) {
    auto m = EnumMetadata<EnumType>::mapping();
    for (auto const& [val, str] : m) {
        if (str == label) {
            return static_cast<EnumType>(val);
        }
    }
    // Fallback to integer conversion if it's a numeric string
    try {
        return static_cast<EnumType>(std::stoi(label));
    } catch (...) {
        // Fallback or throw
    }
    throw std::runtime_error("Unknown enum label: " + label + " for enum: " + EnumMetadata<EnumType>::name());
}

/**
 * @brief Helper to convert an enum value to its string label.
 */
template<typename EnumType>
inline std::string enum_to_string(const EnumType& val) {
    auto m = EnumMetadata<EnumType>::mapping();
    auto it = m.find(static_cast<int>(val));
    if (it != m.end()) {
        return it->second;
    }
    return std::to_string(static_cast<int>(val));
}

/**
 * @brief Helper to generate a documentation string for an enum.
 */
template<typename EnumType>
inline std::string get_enum_doc() {
    auto m = EnumMetadata<EnumType>::mapping();
    std::string doc = "";
    for (auto const& [val, label] : m) {
        if (!doc.empty()) doc += ", ";
        doc += std::to_string(val) + "=" + label;
    }
    return doc;
}

} // namespace d4
