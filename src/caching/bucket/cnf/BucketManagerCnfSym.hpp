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

#include "BucketInConstruction.hpp"
#include "BucketManagerCnf.hpp"
#include "BucketSortInfo.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/exceptions/BucketException.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class BucketManagerCnfSym : public BucketManagerCnf {
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
  /**
     Function called in order to initialized variables before using

     @param[in] occM, the CNF occurrence manager
     @param[in] mdStore, the storing mode for the clause
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
  */
  BucketManagerCnfSym(CnfManager& occM, ModeStore mdStore,
                      unsigned long sizeFirstPage,
                      unsigned long sizeAdditionalPage,
                      BucketAllocator* bucketAllocator = new BucketAllocator());

  /**
     Destructor.
   */
  ~BucketManagerCnfSym();

  /**
     Get an index store the distribution information.

     @param[out] inConstruction, place where we store the bucket in
     construction.

     \return the index of a reserved bucket.
   */
  int getIdxBucketSortInfo(BucketInConstruction& inConstruction);

  void showListBucketSort(std::vector<BucketSortInfo>& v, std::ostream& out);

  /**
     Push sorted, use the natural order.

   */
  void pushSorted(unsigned* tab, unsigned pos, unsigned val);

  /**
     It is used in order to construct a sorted residual formula.

     @param[in] l, we considere the clause containing l
     @param[out] inConstruction, place where we store the bucket in
     construction.
     @param[in] repLit, the representation of the literal l in the formula in
     construction.
  */
  void createDistribWrTLit(const Lit& l, BucketInConstruction& inConstruction,
                           const Lit repLit);

  /**
   * @brief   Collects the clause distribution based on the given ordered
   * literals.
   *
   * This function analyzes the ordered literals to construct clause buckets,
   * storing intermediate results in `inConstruction`. Redundant clauses are
   * identified and removed during the process.
   *
   * @param[in]  orderedLiterals   The list of literals in a predefined order.
   * @param[out] inConstruction    The data structure used to build the clause
   * distribution.
   *
   * @return  The number of elements in the final distribution after removing
   * redundant clauses.
   */
  unsigned collectDistrib(std::vector<Lit>& orderedLiterals,
                          BucketInConstruction& inConstruction);

  /**
     Prepare the data to store a new bucket.

     @param[out] inConstruction, place where we store the bucket in
     construction.
   */
  void initSortBucket(BucketInConstruction& inConstruction);

  /**
     Compute the number of bytes requiered to store the data.
   */
  unsigned computeNeededBytes(unsigned nBda, unsigned nbD, unsigned nbEltData,
                              unsigned nbEltDist);

  /**
     Store the variables respecting the information of size concerning the type
     T to encode each elements and returns the pointer just after the end of the
     data.

     @param[in] data, the place where we print the data.
     @param[in] component, the set of variables.
   */
  template <typename U>
  void* storeVariables(void* data, std::vector<Var>& component);

  /**
   Store the variables respecting the information of size concerning the type T
   to encode each elements and returns the pointer just after the end of the
   data.

   @param[in] data, the place where we store the information.
   @param[out] inConstruction, place where we store the bucket in construction.
 */
  template <typename U>
  void* storeDistribInfo(void* data, BucketInConstruction& inConstruction);

  /**
     Store the formula representation respecting the information of size
     concerning the type T to encode each elements and returns the pointer just
     after the end of the data.

     Information about the formula is store in member variables:
      - m_sizeDistrib
      - m_distrib

     @param[in] data, the place where we store the information
     @param[in] component, is the set of variables.
     @param[out] inConstruction, place where we store the bucket in
     construction.

     \return a pointer to the end of the data we added
  */
  template <typename U>
  void* storeClauses(void* data, std::span<const Var> component,
                     BucketInConstruction& inConstruction);
  /**
     Compute from the m_distribDiffSize the number of different size and the
     maximum size.

     @param[out] maxNbSizeDistr, the clause size with the maximum number of
     elements.
     @parar[out] largestSizeClause, store the size of the largest clause.
     @param[out] nbDiffClauseSize, the number of different size.
     @param[out] nbLit, the number of literals in the distribution.
     @param[out] inConstruction, place where we store the bucket in
     construction.
   */
  void getInfoDistributionSize(unsigned& maxNbSizeClause,
                               unsigned& largestSizeClause,
                               unsigned& nbDiffClauseSize, unsigned& nbLit,
                               BucketInConstruction& inConstruction);

  /**
   * @brief   Sorts the literals associated with a given set of variables.
   *
   * The function processes the input component (a set of variables) by:
   * - Sorting the variables according to their literal order.
   * - For each variable, adding the positive literal first, followed by the
   * negative literal.
   *
   * @param[in] component   The set of variables to process.
   * @param[out] orderedLits   The resulting list of literals, sorted according
   * to the described order.
   */
  void varToSortedLiterals(std::span<const Var> component,
                           std::vector<Lit>& orderedLits);

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables.
     @param[out] tmpFormula, the place where is stored the formula.
     @param[out] szTmpFormula, to collect the size of the stored formula.
  */
  void storeFormula(std::span<const Var> component, DataBucket& b) override;
};
}  // namespace d4
