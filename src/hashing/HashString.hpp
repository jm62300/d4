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

#include <sys/types.h>

#include <cstddef>

#include "rapidhash.h"

namespace d4 {
class HashString {
 public:
  /**
   * @brief Hash the given byte string. The cache keys are numerous and
   * generally short, hence rapidhash with the bucket meta-data (info) folded
   * in as the seed: one single pass over the data.
   */
  inline uint64_t hash(char* key, unsigned len, uint64_t info) {
    return rapidhash_withSeed(key, len, info);
  }  // hash
};
}  // namespace d4
