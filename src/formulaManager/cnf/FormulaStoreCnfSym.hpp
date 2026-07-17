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

#include <algorithm>

#include "FormulaStoreCnf.hpp"
#include "src/caching/bucket/cnf/BucketInConstruction.hpp"
#include "src/caching/bucket/cnf/BucketSortInfo.hpp"
#include "src/exceptions/BucketException.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class FormulaStoreCnfSym : public FormulaStoreCnf {
 private:
  std::vector<BucketSortInfo> m_vecBucketSortInfo;
  int m_unusedBucket;
  std::vector<unsigned long int> m_mapVar;

  std::vector<int> m_mustUnMark;
  std::vector<int> m_markIdx;
  std::vector<unsigned> m_idInVecBucket;

  BucketInConstruction m_inConstruction;
  unsigned* m_offsetClauses;

  std::vector<unsigned> m_storeNbClause;
  std::vector<unsigned> m_litDegree;

  std::vector<unsigned long> m_stampDegreeVector;
  unsigned long m_stampDegreeIndex;
  unsigned* m_memoryPlaceWrtSizeClause;

 public:
  FormulaStoreCnfSym(CnfManager& occM, ModeStore mdStore);
  ~FormulaStoreCnfSym();

  int getIdxBucketSortInfo(BucketInConstruction& inConstruction);
  void showListBucketSort(std::vector<BucketSortInfo>& v, std::ostream& out);
  void pushSorted(unsigned* tab, unsigned pos, unsigned val);

  void createDistribWrTLit(const Lit& l, BucketInConstruction& inConstruction,
                           const Lit repLit);

  unsigned collectDistrib(std::vector<Lit>& orderedLiterals,
                          BucketInConstruction& inConstruction);

  void initSortBucket(BucketInConstruction& inConstruction);
  unsigned computeNeededBytes(unsigned nBda, unsigned nbD, unsigned nbEltData,
                              unsigned nbEltDist);

  template <typename U>
  void* storeVariables(void* data, std::vector<Var>& component);

  template <typename U>
  void* storeDistribInfo(void* data, BucketInConstruction& inConstruction);

  template <typename U>
  void* storeClauses(void* data, std::span<const Var> component,
                     BucketInConstruction& inConstruction);

  void getInfoDistributionSize(unsigned& maxNbSizeClause,
                               unsigned& largestSizeClause,
                               unsigned& nbDiffClauseSize, unsigned& nbLit,
                               BucketInConstruction& inConstruction);

  void varToSortedLiterals(std::span<const Var> component,
                           std::vector<Lit>& orderedLits);

  void storeFormula(std::span<const Var> component, DataBucket& b,
                    BucketAllocator& alloc) override;
};

}  // namespace d4
