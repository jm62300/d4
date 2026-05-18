#pragma once

#include <string>
#include <map>
#include <vector>

#include "src/exceptions/FactoryException.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/EnumMetadata.hpp"

namespace d4 {

enum ModeStore { CACHE_ALL, CACHE_NB, CACHE_NT };
enum ClauseRepresentation { CACHE_CLAUSE, CACHE_INDEX, CACHE_SYM, CACHE_COMBI };

template <>
struct EnumMetadata<ModeStore> {
  static std::string name() { return "ModeStore"; }
  static std::map<int, std::string> mapping() {
    return {{CACHE_ALL, "all"}, {CACHE_NB, "not-binary"}, {CACHE_NT, "not-touched"}};
  }
};

template <>
struct EnumMetadata<ClauseRepresentation> {
  static std::string name() { return "ClauseRepresentation"; }
  static std::map<int, std::string> mapping() {
    return {{CACHE_CLAUSE, "clause"}, {CACHE_INDEX, "index"}, {CACHE_SYM, "sym"}, {CACHE_COMBI, "combi"}};
  }
};

class OptionBucketManager : public OptionGroup {
 public:
  OptionBucketManager(const std::string& name = "bucket", const std::string& description = "Bucket manager options")
      : OptionGroup(name, description) {}

  /** @brief The strategy used to store the clause in a bucket (all, not-binary and not-touched). */
  Option<ModeStore> modeStore{"modeStore", "The strategy used to store the clause in a bucket", CACHE_NT};
  /** @brief The way the clause are represented in the cache (combi, sym, clause and index). */
  Option<ClauseRepresentation> clauseRepresentation{"clauseRepresentation", "The way the clause are represented in the cache", CACHE_CLAUSE};
  /** @brief The block size of memory allocated for the first page. */
  Option<unsigned long> sizeFirstPage{"sizeFirstPage", "The block size of memory allocated for the first page", 1UL << 32};
  /** @brief The block size of memory allocated for the next page. */
  Option<unsigned long> sizeAdditionalPage{"sizeAdditionalPage", "The block size of memory allocated for the next page", 1UL << 29};
  /** @brief Limit for symmetry caching. */
  Option<unsigned> limitVarSym{"limitVarSym", "Limit for symmetry caching", 20};
  /** @brief Limit for index caching. */
  Option<unsigned> limitVarIndex{"limitVarIndex", "Limit for index caching", 2000};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&modeStore, (OptionBase*)&clauseRepresentation,
            (OptionBase*)&sizeFirstPage, (OptionBase*)&sizeAdditionalPage,
            (OptionBase*)&limitVarSym, (OptionBase*)&limitVarIndex};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionBucketManager& dt) {
    out << " Option BucketManager:"
        << " storage(" << dt.modeStore.getValueAsString() << ") "
        << " representation(" << dt.clauseRepresentation.getValueAsString() << ") "
        << " size_first_page(" << dt.sizeFirstPage.get() << ")"
        << " size_additional_page(" << dt.sizeAdditionalPage.get() << ")";

    if (dt.clauseRepresentation.get() == CACHE_COMBI)
      out << " limit #var sym(" << dt.limitVarSym.get() << ")"
          << " limit #var index (" << dt.limitVarIndex.get() << ")";

    return out;
  }
};

}  // namespace d4
