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

#include <bitset>
#include <iostream>
#include <vector>

#include "CnfManager.hpp"
#include "FormulaStore.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/bucket/BucketAllocator.hpp"
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class FormulaStoreCnf : public FormulaStore {
 protected:
  CnfManager& m_specManager;

  ModeStore m_modeStore;
  unsigned m_nbClauseCnf;
  unsigned m_nbVarCnf;
  unsigned m_maxSizeClause;

  std::vector<bool> m_varInComponent;
  std::vector<int> m_idxClauses;

 public:
  FormulaStoreCnf(CnfManager& occM, ModeStore mdStore);

  virtual ~FormulaStoreCnf() = default;

  virtual void storeFormula(std::span<const Var> component, DataBucket& b,
                            BucketAllocator& alloc) = 0;

  void collectIdActiveClauses(std::span<const Var> component,
                              std::vector<unsigned>& idxClauses);

  inline bool canSkipLit(const Lit& l) {
    if (m_modeStore == CACHE_NT)
      return m_specManager.getNbRemainingInitNotBinaryClause(l) == 0;
    return false;
  }

  inline bool isKeptClause(int idx) {
    switch (m_modeStore) {
      case CACHE_NT:
        return m_specManager.getNbUnsat(idx);
      case CACHE_NB:
        return m_specManager.getClause(idx).size() > 2;
      default:
        return true;
    }
  }

  // Utility: number of bits needed to encode v (~log2(v)+1).
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

  // Utility: minimum number of bytes needed to encode v.
  inline int nbOctetToEncodeInt(unsigned int v) {
    if (v < (1 << 8)) return 1;
    if (v < (1 << 16)) return 2;
    return 4;
  }
};

}  // namespace d4
