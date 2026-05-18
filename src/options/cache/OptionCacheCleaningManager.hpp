#pragma once

#include <string>
#include <map>
#include <vector>

#include "src/exceptions/FactoryException.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/EnumMetadata.hpp"

namespace d4 {

enum CacheCleaningStrategy { CACHE_EXPECTATION, CACHE_NONE };

template <>
struct EnumMetadata<CacheCleaningStrategy> {
  static std::string name() { return "CacheCleaningStrategy"; }
  static std::map<int, std::string> mapping() {
    return {{CACHE_EXPECTATION, "expectation"}, {CACHE_NONE, "none"}};
  }
};

class OptionCacheCleaningManager : public OptionGroup {
 public:
  OptionCacheCleaningManager(const std::string& name = "cleaning", const std::string& description = "Cache cleaning options")
      : OptionGroup(name, description) {}

  /** @brief The strategy used to reduce the cache structure [none, expectation]. */
  Option<CacheCleaningStrategy> cacheCleaningStrategy{"strategy", "The strategy used to reduce the cache structure", CACHE_EXPECTATION};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&cacheCleaningStrategy};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionCacheCleaningManager& dt) {
    out << " Option CacheCleaningManager:"
        << " cleaning strategy("
        << dt.cacheCleaningStrategy.getValueAsString()
        << ") ";

    return out;
  }
};
}  // namespace d4
