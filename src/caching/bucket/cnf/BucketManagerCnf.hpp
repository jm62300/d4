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

#include "../../CachedBucket.hpp"
#include "../BucketAllocator.hpp"
#include "../BucketManager.hpp"
#include "src/formulaManager/cnf/CnfManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class BucketManagerCnf : public BucketManager {
 protected:
  CnfManager &m_specManager;

  ModeStore m_modeStore;
  unsigned m_nbClauseCnf;
  unsigned m_nbVarCnf;
  unsigned m_maxSizeClause;

  std::vector<bool> m_varInComponent;
  std::vector<int> m_idxClauses;

 public:
  /**
     Constructor.

     @param[in] occM, the CNF occurrence manager.
     @param[in] mdStore, the storing mode for the clause.
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
     @param[in] bucketAllocator, a bucket allocator.
  */
  BucketManagerCnf(CnfManager &occM, ModeStore mdStore,
                   unsigned long sizeFirstPage,
                   unsigned long sizeAdditionalPage,
                   BucketAllocator *bucketAllocator);

  /**
   * @brief Get the clauses that will be used, that are the clause that
   * respect the modeStore.
   *
   * @param[in] component, the variables in the current component.
   * @param[out] idxClauses, the resulting clauses (index).
   */
  void collectIdActiveClauses(std::vector<Var> &component,
                              std::vector<unsigned> &idxClauses);

  virtual ~BucketManagerCnf() { ; }
  virtual void storeFormula(std::vector<Var> &component, DataBucket &b) = 0;

  inline bool canSkipLit(const Lit &l) {
    if (m_modeStore == CACHE_NT)
      return m_specManager.getNbRemainingInitNotBinaryClause(l) == 0;
    return false;
  }  // canSkipLit

  /**
   * Tell if the clause given as parameter (which is represented by its index
   * in the spec manager) should be considered or not.
   *
   * @param[in] idx, the index of the clause.
   *
   * \return true if the clause is kept, false otherwise.
   */
  inline bool isKeptClause(int idx) {
    switch (m_modeStore) {
      case CACHE_NT:
        return m_specManager.getNbUnsat(idx);
      case CACHE_NB:
        return m_specManager.getClause(idx).size() > 2;
      default:
        return true;
    }
  }  // isKeptClause
};
}  // namespace d4
