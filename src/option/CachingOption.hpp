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

namespace d4 {
namespace caching {
enum StoreStrategy { ALL, NOT_BINARY, NOT_TOUCHED };
enum ReductionStrategy { NONE, EXPECTATION, CACHE, SHARPSAT };
enum CacheRepresentation { LIST, NO_COLLISION };
enum ClauseRepresentationType { CLAUSE, INDEX, SYM, COMBI };

class ClauseRepresentation {
 public:
  ClauseRepresentationType type = CLAUSE;
  unsigned limitVar_sym = 20;
  unsigned limitVar_index = 2000;
};

class CacheReductionStrategy {
 public:
  ReductionStrategy strategy = NONE;
  unsigned long cache_limit = 10UL * (1 << 21);
  unsigned long expectation_limit = 100000;
  double expectation_ratio = 0.3;
};

class CachingOption {
 public:
  CacheRepresentation cache_representation = LIST;
  StoreStrategy store_strategy = NOT_TOUCHED;
  CacheReductionStrategy reduction_strategy;
  ClauseRepresentation clause_representation;

  unsigned long size_first_page = 1UL << 32;
  unsigned long size_additional_page = 1UL << 29;
  unsigned limit_number_variable = 100000;
  unsigned limit_ratio = 0;

  bool activate = true;
};
}  // namespace caching
}  // namespace d4