#pragma once

#include <string>
#include <map>
#include <vector>

#include "src/exceptions/FactoryException.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/EnumMetadata.hpp"
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/options/cache/OptionCacheCleaningManager.hpp"

namespace d4 {

enum CachingMethod { CACHE_NO_COL, CACHE_LIST };

template <>
struct EnumMetadata<CachingMethod> {
  static std::string name() { return "CachingMethod"; }
  static std::map<int, std::string> mapping() {
    return {{CACHE_NO_COL, "no-colission"}, {CACHE_LIST, "list"}};
  }
};

class OptionCacheManager : public OptionGroup {
 public:
  OptionCacheManager(const std::string& name = "cache", const std::string& description = "Cache options")
      : OptionGroup(name, description) {}

  Option<CachingMethod> cachingMethod{"cachingMethod", "The way the collision are handled", CACHE_LIST};
  OptionBucketManager optionBucketManager{"bucket", "Bucket manager options"};
  OptionCacheCleaningManager optionCacheCleaningManager{"cleaning", "Cache cleaning options"};
  Option<bool> isActivated{"isActivated", "Activate or not the cache", true};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&cachingMethod, (OptionBase*)&optionBucketManager,
            (OptionBase*)&optionCacheCleaningManager, (OptionBase*)&isActivated};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionCacheManager& dt) {
    out << " Option Cache:"
        << " is activated?(" << dt.isActivated.get() << ")"
        << " caching method(" << dt.cachingMethod.getValueAsString() << ")"
        << " " << dt.optionCacheCleaningManager
        << " " << dt.optionBucketManager;
    return out;
  }
};
}  // namespace d4