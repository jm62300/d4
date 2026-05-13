#pragma once
#include <string>
#include <map>

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
