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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <bitset>
#include <iostream>
#include <vector>

#include "src/problem/ProblemTypes.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"
#include "src/utils/Enum.hpp"

#include "../BucketAllocator.hpp"
#include "../BucketManager.hpp"
#include "../CachedBucket.hpp"

namespace d4 {
template <class T> class BucketManager;
template <class T> class Cache;
template <class T> class BucketManagerCnf;

template <class T> class BucketManagerCnf : public BucketManager<T> {
protected:
  SpecManagerCnf &specManager;

  ModeStore modeStore;
  unsigned nbClauseCnf;
  unsigned nbVarCnf;
  unsigned m_maxSizeClause;

  std::vector<bool> m_varInComponent;
  std::vector<int> m_idxClauses;

  using BucketManager<T>::m_cache;
  using BucketManager<T>::m_bucketAllocator;

public:
  /**
     Constructor.

     @param[in] occM, the CNF occurrence manager.
     @param[in] cache, the cache the bucket is linked with.
     @param[in] mdStore, the storing mode for the clause.
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
     @param[in] bucketAllocator, a bucket allocator.
  */
  BucketManagerCnf(SpecManagerCnf &occM, Cache<T> *cache, ModeStore mdStore,
                   unsigned long sizeFirstPage,
                   unsigned long sizeAdditionalPage,
                   BucketAllocator *bucketAllocator)
      : specManager(occM) {
    this->m_cache = cache;
    this->m_bucketAllocator = bucketAllocator;
    modeStore = mdStore;
    nbClauseCnf = occM.getNbClause();
    nbVarCnf = occM.getNbVariable();
    m_maxSizeClause = occM.getMaxSizeClause();
    m_varInComponent.resize(nbVarCnf, false);

    m_bucketAllocator->init(
        sizeFirstPage, sizeAdditionalPage, [this](char *data, int posInHash) {
          CachedBucket<T> &v = m_cache->getHashTable()[posInHash];
          v.getDataInfo().info1 = 0;
        });
  } // BucketManager

  virtual ~BucketManagerCnf() { ; }
  virtual void storeFormula(std::vector<Var> &component,
                            CachedBucket<T> &b) = 0;

  /**
     Tell if the clause given as parameter (which is represented by its index in
     the spec manager) should be considered or not.

     @param[in] idx, the index of the clause.

     \return true if the clause is kept, false otherwise.
   */
  bool isKeptClause(int idx) {
    switch (modeStore) {
    case NT:
      return specManager.getNbUnsat(idx);
    case NB:
      return specManager.getClause(idx).size() > 2;
    default:
      return true;
    }
  } // isKeptClause

  /**
     Get the clauses that will be used, that are the clause that respect the
     modeStore.

     @param[in] component, the variables in the current component.
     @param[out] idxClauses, the resulting clauses (index).
  */
  void collectIdActiveClauses(std::vector<Var> &component,
                              std::vector<unsigned> &idxClauses) {
    // collect the clauses
    idxClauses.resize(0);
    if (modeStore == ALL)
      specManager.getCurrentClauses(idxClauses, component);
    else
      specManager.getCurrentClausesNotBin(idxClauses, component);

    unsigned i, j;
    for (i = j = 0; i < idxClauses.size(); i++) {
      if (!isKeptClause(idxClauses[i]))
        continue;
      idxClauses[j++] = idxClauses[i];
    }
    idxClauses.resize(j);
  } // collectIdActiveClauses
};
} // namespace d4
