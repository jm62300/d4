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

#include "src/options/cache/OptionCacheManager.hpp"

namespace d4 {
struct ConfigurationCache {
  /** @brief The way the collision are handled (no-collision or list). */
  CachingMethod cachingMethod = CACHE_LIST;
  /** @brief The strategy used to reduce the cache structure [none, expectation, cache or sharpSAT]. */
  CacheCleaningStrategy cacheCleaningStrategy = CACHE_EXPECTATION;
  /** @brief The strategy used to store the clause in a bucket (all, not-binary and not-touched). */
  ModeStore modeStore = CACHE_NT;
  /** @brief The way the clause are represented in the cache (combi, sym, clause and index). */
  ClauseRepresentation clauseRepresentation = CACHE_CLAUSE;

  /** @brief Activate or not the cache. */
  bool isActivated = true;
  /** @brief The block size of memory allocated for the first page of the cache structure. */
  unsigned long sizeFirstPage = 1UL << 32;
  /** @brief The block size of memory allocated for the next page of the cache structure. */
  unsigned long sizeAdditionalPage = 1UL << 29;
  /** @brief In the mixed strategy, if we have less than a given number of variable then we use the symmetry caching representation. */
  unsigned limitVarSym = 20;
  /** @brief In the mixed strategy, if we have more than a given number of variable then we use the index caching representation. */
  unsigned limitVarIndex = 2000;
};
}  // namespace d4