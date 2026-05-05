/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#pragma once

#include <string>

#include "src/exceptions/FactoryException.hpp"

namespace d4 {

enum ModeStore { CACHE_ALL, CACHE_NB, CACHE_NT };
enum ClauseRepresentation { CACHE_CLAUSE, CACHE_INDEX, CACHE_SYM, CACHE_COMBI };

class ModeStoreManager {
 public:
  static std::string getModeStore(const ModeStore& m) {
    if (m == CACHE_ALL) return "all";
    if (m == CACHE_NB) return "not-binary";
    if (m == CACHE_NT) return "not-touched";

    throw(FactoryException("ModeStore unknown", __FILE__, __LINE__));
  }  // getModeStoreName

  static ModeStore getModeStore(const std::string& m) {
    if (m == "all") return CACHE_ALL;
    if (m == "not-touched") return CACHE_NT;
    if (m == "not-binary") return CACHE_NB;
    throw(FactoryException("ModeStore unknown", __FILE__, __LINE__));
  }  // getModeStoreName
};

class ClauseRepresentationManager {
 public:
  static std::string getClauseRepresentation(const ClauseRepresentation& c) {
    if (c == CACHE_CLAUSE) return "clause";
    if (c == CACHE_INDEX) return "index";
    if (c == CACHE_SYM) return "sym";
    if (c == CACHE_COMBI) return "combi";
    throw(FactoryException("ClauseRepresentation unknown", __FILE__, __LINE__));
  }  // getClauseRepresentationName

  static ClauseRepresentation getClauseRepresentation(const std::string& c) {
    if (c == "clause") return CACHE_CLAUSE;
    if (c == "index") return CACHE_INDEX;
    if (c == "sym") return CACHE_SYM;
    if (c == "combi") return CACHE_COMBI;
    throw(FactoryException("ClauseRepresentation unknown", __FILE__, __LINE__));
  }  // getClauseRepresentationName
};

class OptionBucketManager {
 public:
  /** @brief The strategy used to store the clause in a bucket (all, not-binary and not-touched). */
  ModeStore modeStore;
  /** @brief The way the clause are represented in the cache (combi, sym, clause and index). */
  ClauseRepresentation clauseRepresentation;
  /** @brief The block size of memory allocated for the first page of the cache structure. */
  unsigned long sizeFirstPage;
  /** @brief The block size of memory allocated for the next page of the cache structure. */
  unsigned long sizeAdditionalPage;
  /** @brief In the mixed strategy, if we have less than a given number of variable then we use the symmetry caching representation. */
  unsigned limitNbVarSym = 0;
  /** @brief In the mixed strategy, if we have more than a given number of variable then we use the index caching representation. */
  unsigned limitNbVarIndex = 0;

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionBucketManager& dt) {
    out << " Option BucketManager:"
        << " storage(" << ModeStoreManager::getModeStore(dt.modeStore) << ") "
        << " representation("
        << ClauseRepresentationManager::getClauseRepresentation(
               dt.clauseRepresentation)
        << ") "
        << " size_first_page(" << dt.sizeFirstPage << ")"
        << " size_additional_page(" << dt.sizeAdditionalPage << ")";

    if (dt.clauseRepresentation == CACHE_COMBI)
      out << " limit #var sym(" << dt.limitNbVarSym << ")"
          << " limit #var index (" << dt.limitNbVarIndex << ")";

    return out;
  }  // <<
};

}  // namespace d4
