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
#include <vector>

#include "src/formulaManager/cnf/CnfManager.hpp"

namespace d4 {

class BucketInConstruction {
 public:
  unsigned *distrib;
  unsigned *shiftedIndexClause;
  unsigned *shiftedSizeClause;
  unsigned *sizeClauses;
  unsigned *distribDiffSize;
  bool *markedAsRedundant;

  // the clause sizes s such that distribDiffSize[s] > 0.
  std::vector<unsigned> presentSizes;

  unsigned nbClauseInDistrib;
  unsigned sizeDistrib;
  unsigned capacityDistrib;
  unsigned maxSizeClause;

  BucketInConstruction();
  BucketInConstruction(CnfManager &occM);
  ~BucketInConstruction();
  void reinit();

  /**
   * @brief Increment distribDiffSize for the given clause size, keeping
   * presentSizes in sync.
   */
  inline void addClauseSize(unsigned sz) {
    if (!distribDiffSize[sz]++) presentSizes.push_back(sz);
  }  // addClauseSize

  /**
   * @brief Sort presentSizes in ascending order (the list is expected to be
   * very small, hence the insertion sort).
   */
  inline void sortPresentSizes() {
    for (unsigned i = 1; i < presentSizes.size(); i++) {
      unsigned val = presentSizes[i], j = i;
      for (; j > 0 && presentSizes[j - 1] > val; j--)
        presentSizes[j] = presentSizes[j - 1];
      presentSizes[j] = val;
    }
  }  // sortPresentSizes
};
}  // namespace d4
