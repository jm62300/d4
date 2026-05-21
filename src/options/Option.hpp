#pragma once
#include <optree/Option.hpp>
#include "src/options/OptionBase.hpp"

namespace d4 {
template <typename T>
using Option = optree::Option<T>;
}
