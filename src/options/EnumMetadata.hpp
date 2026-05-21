#pragma once
#include <optree/EnumMetadata.hpp>
#include <type_traits>

namespace d4 {

// Traditional primary template that D4's existing files specialize.
template <typename EnumType>
struct EnumMetadata {
    static std::map<int, std::string> mapping() { return {}; }
    static std::string name() { return "UnknownEnum"; }
};

using optree::resolve_enum;
using optree::enum_to_string;
using optree::get_enum_doc;

} // namespace d4

namespace optree {

template <typename T>
struct EnumMetadata<T, std::enable_if_t<std::is_enum_v<T>>> : public d4::EnumMetadata<T> {};

} // namespace optree
