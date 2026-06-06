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

#include <cassert>
#include <iostream>
#include <span>

#include "../CachedBucket.hpp"
#include "BucketAllocator.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {

class BucketManager {
 protected:
  FormulaManager& m_formulaManager;
  BucketAllocator* m_bucketAllocator;

 public:
  BucketManager(FormulaManager& fm, BucketAllocator* alloc)
      : m_formulaManager(fm), m_bucketAllocator(alloc) {}

  ~BucketManager() {
    if (m_bucketAllocator->getCleanup()) delete m_bucketAllocator;
  }

  /**
   * @brief Collect the bucket associated to the set of variable given in
   * parameter. Delegates to the formula manager's storeFormula.
   *
   * @param[in] component, the variable belonging to the connected component
   * @param[in] bucket is the place where are stored the information.
   */
  inline void collectBucket(std::span<const Var> component,
                            DataBucket& bucket) {
    m_formulaManager.storeFormula(component, bucket, *m_bucketAllocator);
  }

  inline unsigned long int usedMemory() {
    return m_bucketAllocator->usedMemory();
  }

  inline bool getComsumedMemory() {
    return m_bucketAllocator->getComsumedMemory();
  }

  inline void reinitComsumedMemory() {
    m_bucketAllocator->reinitComsumedMemory();
  }

  inline void releaseMemory(char* m, unsigned size) {
    m_bucketAllocator->releaseMemory(m, size);
  }

  inline double remainingMemory() {
    return m_bucketAllocator->remainingMemory();
  }

  // Kept as static utility, still used by FormulaStoreCnfCl/Sym.
  inline static unsigned nbBitUnsigned(unsigned v) {
    const unsigned int b[] = {0x2, 0xC, 0xF0, 0xFF00, 0xFFFF0000};
    const unsigned int S[] = {1, 2, 4, 8, 16};
    unsigned int r = 0;
    for (int i = 4; i >= 0; i--) {
      if (v & b[i]) {
        v >>= S[i];
        r |= S[i];
      }
    }
    return r + 1;
  }
};

}  // namespace d4
