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

#include <algorithm>
#include <cstring>

#include "FormulaStoreCnf.hpp"
#include "src/caching/bucket/cnf/BucketInConstruction.hpp"
#include "src/caching/bucket/cnf/BucketSortInfo.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

struct AllocSizeInfo {
  unsigned nbBitEltVar = 0;
  unsigned nbByteStoreVar = 0;
  unsigned nbByteStoreFormula = 0;
  unsigned nbBitStoreLit = 0;
  unsigned totalByte = 0;

  inline void display() {
    std::cout << "nb bit var: " << nbBitEltVar << "\n"
              << "nb bit lit: " << nbBitStoreLit << "\n"
              << "nb byte store var: " << nbByteStoreVar << "\n"
              << "nb byte store formula: " << nbByteStoreFormula << "\n"
              << "total: " << totalByte << "\n";
  }
};

class FormulaStoreCnfCl : public FormulaStoreCnf {
 private:
  std::vector<BucketSortInfo> m_vecBucketSortInfo;
  int m_unusedBucket;
  std::vector<unsigned long int> m_mapVar;

  std::vector<int> m_mustUnMark;
  std::vector<int> m_markIdx;
  std::vector<unsigned> m_idInVecBucket;

  BucketInConstruction m_inConstruction;
  unsigned* m_memoryPosWrtClauseSize;
  unsigned* m_offsetClauses;

 protected:
  int getIdxBucketSortInfo(BucketInConstruction& inConstruction);
  void pushSorted(unsigned* tab, unsigned pos, unsigned val);
  void createDistribWrTLit(const Lit& l, BucketInConstruction& inConstruction);

  unsigned collectDistrib(std::span<const Var> component,
                          BucketInConstruction& inConstruction);

  void initSortBucket(BucketInConstruction& inConstruction);
  void showListBucketSort(std::vector<BucketSortInfo>& v, std::ostream& out);

  char* addElementInData(char* p, unsigned val, unsigned nbBit,
                         unsigned& remainingBit);

  char* storeVariables(AllocSizeInfo& info, char* data,
                       std::span<const Var> component);

  AllocSizeInfo computeNeededBytes(std::span<const Var> component,
                                   BucketInConstruction& inConstruction);

  char* storeClauses(AllocSizeInfo& info, char* data,
                     std::span<const Var> component,
                     BucketInConstruction& inConstruction);

 public:
  FormulaStoreCnfCl(CnfManager& occM, ModeStore mdStore);
  ~FormulaStoreCnfCl();

  void storeFormula(std::span<const Var> component, DataBucket& b,
                    BucketAllocator& alloc) override;
};

}  // namespace d4
