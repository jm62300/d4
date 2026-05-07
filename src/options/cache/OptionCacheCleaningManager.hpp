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

class CacheCleaningStrategyManager {
 public:
  static std::string getCacheCleaningStrategy(const CacheCleaningStrategy& m) {
    if (m == CACHE_EXPECTATION) return "expectation";
    if (m == CACHE_NONE) return "none";

    throw(
        FactoryException("CacheCleaningStrategy unknown", __FILE__, __LINE__));
  }

  static CacheCleaningStrategy getCacheCleaningStrategy(const std::string& m) {
    if (m == "expectation") return CACHE_EXPECTATION;
    if (m == "none") return CACHE_NONE;
    throw(
        FactoryException("CacheCleaningStrategy unknown", __FILE__, __LINE__));
  }

  static std::map<int, std::string> getMapping() {
    return {{CACHE_EXPECTATION, "expectation"}, {CACHE_NONE, "none"}};
  }
};

template <>
struct EnumMetadata<CacheCleaningStrategy> {
  static std::string name() { return "CacheCleaningStrategy"; }
  static std::map<int, std::string> mapping() { return CacheCleaningStrategyManager::getMapping(); }
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
        << CacheCleaningStrategyManager::getCacheCleaningStrategy(dt.cacheCleaningStrategy.get())
        << ") ";

    return out;
  }
};
}  // namespace d4
