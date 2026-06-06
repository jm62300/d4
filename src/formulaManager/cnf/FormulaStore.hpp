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

#include <span>

#include "src/caching/CachedBucket.hpp"
#include "src/caching/bucket/BucketAllocator.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class FormulaStore {
 public:
  virtual ~FormulaStore() = default;

  /**
   * @brief Serialize the current formula state for the given connected component
   * into a compact byte representation suitable for caching.
   *
   * The representation is sound: two calls with equivalent (same model count)
   * residual formulas produce identical bytes.
   *
   * @param[in] component variables in the current connected component.
   * @param[out] b the bucket that receives the serialized representation.
   * @param[in] alloc the allocator used to obtain raw memory for the bytes.
   */
  virtual void storeFormula(std::span<const Var> component, DataBucket& b,
                            BucketAllocator& alloc) = 0;
};

}  // namespace d4
