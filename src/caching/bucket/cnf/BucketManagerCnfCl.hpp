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
#include <bitset>
#include <cstring>

#include "BucketInConstruction.hpp"
#include "BucketManagerCnf.hpp"
#include "BucketSortInfo.hpp"
#include "src/caching/CacheManager.hpp"
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

class BucketManagerCnfCl : public BucketManagerCnf {
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
  /**
   * Get an index to store the distribution information.
   *
   * @param[out] inConstruction, place where we store the bucket in
   * construction.
   *
   * \return the index of a reserved bucket.
   */
  int getIdxBucketSortInfo(BucketInConstruction& inConstruction);

  /**
   * Push sorted, use the natural order.
   */
  void pushSorted(unsigned* tab, unsigned pos, unsigned val);

  /**
   *  It is used in order to construct a sorted residual formula.
   *
   *  @param[in] l, we considere the clause containing l
   *  @param[out] inConstruction, place where we store the bucket in
   *  construction.
   */
  void createDistribWrTLit(const Lit& l, BucketInConstruction& inConstruction);

  /**
   * Collect the clause distribution. The result is stored in distrib.
   *
   * @param[in] component, the set of variables we consider.
   * @param[out] inConstruction, place where we store the bucket in
   * construction.
   *
   * \return the number of elements we have in the distribution once the
   * redundant clauses have been removed.
   */
  unsigned collectDistrib(std::span<const Var> component,
                          BucketInConstruction& inConstruction);

  /**
   * Prepare the data to store a new bucket.
   *
   * @param[out] inConstruction, place where we store the bucket in
   * construction.
   */
  void initSortBucket(BucketInConstruction& inConstruction);

  /**
   * @brief Display the bucket in construction (for debugging purpose).
   *
   * @param v
   * @param out
   */
  void showListBucketSort(std::vector<BucketSortInfo>& v, std::ostream& out);

  /**
   * @brief Add nbBit from val in the data vector regarding the number of
   * available bits.
   *
   * @param[in] p is the place where are put the bits.
   * @param[in] val gives the bits.
   * @param[in] nbBit gives the number of bits from val we want to consider.
   * @param[out] remainingBit gives the number of remaining bit in p[0].
   * @return the character in p we reach after adding the bits.
   */
  char* addElementInData(char* p, unsigned val, unsigned nbBit,
                         unsigned& remainingBit);

  /**
   * @brief Store the variables respecting the information of size concerning
   * the type T to encode each elements and returns the pointer just after the
   * end of the data.
   *
   * @param info gives the information about the way of storing the data.
   * @param data is a pointer to memory where we want to store the data.
   * @param component is the set of variables we want to store.
   * @return the remaining data.
   */
  char* storeVariables(AllocSizeInfo& info, char* data,
                       std::span<const Var> component);

  /**
   * @brief Search for the number of bytes needed to store the different element
   * of the bucket.
   *
   * @param component is the set of variables.
   * @param inConstruction is the bucket that has been constructed.
   * @return an AllocSizeInfo structure with the requiered information.
   */
  AllocSizeInfo computeNeededBytes(std::span<const Var> component,
                                   BucketInConstruction& inConstruction);

  /**
     Store the formula representation respecting the information of size
     concerning the type T to encode each elements and returns the pointer
     just after the end of the data.

     Information about the formula is store in member variables:
      - m_sizeDistrib
      - m_distrib

     @param[in] data, the place where we store the information
     @param[in] component, is the set of variables.
     @param[out] inConstruction, place where we store the bucket in
     construction.

     \return a pointer to the end of the data we added
  */
  char* storeClauses(AllocSizeInfo& info, char* data,
                     std::span<const Var> component,
                     BucketInConstruction& inConstruction);

 public:
  /**
   * @brief Constructor, it consists mainly in initializing all the attributs
   * needed.
   *
   * @param[in] occM, the CNF occurrence manager
   * @param[in] mdStore, the storing mode for the clause
   * @param[in] sizeFirstPage, the amount of bytes for the first page.
   * @param[in] sizeAdditionalPage, the amount of bytes for the additional
   * pages.
   */
  BucketManagerCnfCl(CnfManager& occM, ModeStore mdStore,
                     unsigned long sizeFirstPage,
                     unsigned long sizeAdditionalPage,
                     BucketAllocator* bucketAllocator = new BucketAllocator());

  /**
   * Destructor.
   */
  ~BucketManagerCnfCl();

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables.
     @param[out] tmpFormula, the place where is stored the formula.
     @param[out] szTmpFormula, to collect the size of the stored formula.
  */
  void storeFormula(std::span<const Var> component, DataBucket& b) override;
};
}  // namespace d4
