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

#include <algorithm>
#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include "BucketInConstruction.hpp"
#include "BucketSortInfo.hpp"
#include "DataInfoCnfCl.hpp"

#include "src/caching/Cache.hpp"
#include "src/caching/cnf/BucketManagerCnf.hpp"
#include "src/exceptions/BucketException.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/utils/Enum.hpp"

namespace d4 {
template <class T> class BucketManagerCnf;

template <class T> class BucketManagerCnfCl : public BucketManagerCnf<T> {
private:
  /**
   * @brief Compute the number of bit needed to encode an unsigned given in
   * parameter.
   *
   * @param v is the value we search for its number of bits.
   * @return the number of bit needed to encode val (~log2(val)).
   */
  inline static unsigned nbBitUnsigned(unsigned v) {
    const unsigned int b[] = {0x2, 0xC, 0xF0, 0xFF00, 0xFFFF0000};
    const unsigned int S[] = {1, 2, 4, 8, 16};
    int i;

    unsigned int r = 0;      // result of log2(v) will go here
    for (i = 4; i >= 0; i--) // unroll for speed...
    {
      if (v & b[i]) {
        v >>= S[i];
        r |= S[i];
      }
    }

    return r + 1;
  } // nbBitUnsigned

  struct AllocSizeInfo {
    u_int8_t config = 0;

    unsigned nbBitEltVar = 0;
    unsigned nbByteStoreVar = 0;

    unsigned nbBitEltDistribSizeCl = 0;
    unsigned nbBitEltDistribNbCl = 0;
    unsigned nbByteStoreDistrib = 0;
    unsigned sizeDistrib = 0;

    unsigned nbBitEltClauses = 0;
    unsigned nbByteStoreClauses = 0;

    unsigned totalByte = 0;

    inline void initDistrib(unsigned maxLenght, unsigned maxSize,
                            unsigned count) {
      nbBitEltDistribSizeCl = nbBitUnsigned(maxSize);
      nbBitEltDistribNbCl = nbBitUnsigned(maxLenght);
      nbByteStoreDistrib =
          1 +
          (((count * (nbBitEltDistribSizeCl + nbBitEltDistribNbCl)) - 1) >> 3);

      std::cout << maxLenght << " " << maxSize << " " << nbBitEltDistribNbCl
                << " " << nbBitEltDistribSizeCl << " " << count << " "
                << nbByteStoreDistrib << "\n";
    }
  };

  std::vector<BucketSortInfo> m_vecBucketSortInfo;
  int m_unusedBucket;
  std::vector<unsigned long int> m_mapVar;

  std::vector<int> m_mustUnMark;
  std::vector<int> m_markIdx;
  std::vector<unsigned> m_idInVecBucket;

  BucketInConstruction m_inConstruction;
  unsigned *m_offsetClauses;

  // using: variables
  using BucketManagerCnf<T>::specManager;
  using BucketManagerCnf<T>::nbClauseCnf;
  using BucketManagerCnf<T>::nbVarCnf;
  using BucketManagerCnf<T>::m_maxSizeClause;
  using BucketManagerCnf<T>::m_idxClauses;
  using BucketManagerCnf<T>::modeStore;
  using BucketManagerCnf<T>::m_bucketAllocator;

  // using: functions
  using BucketManagerCnf<T>::isKeptClause;

public:
  /**
     Function called in order to initialized variables before using

     @param[in] occM, the CNF occurrence manager
     @param[in] cache, the cache the bucket is linked with.
     @param[in] mdStore, the storing mode for the clause
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
  */
  BucketManagerCnfCl(SpecManagerCnf &occM, Cache<T> *cache, ModeStore mdStore,
                     unsigned long sizeFirstPage,
                     unsigned long sizeAdditionalPage,
                     BucketAllocator *bucketAllocator = new BucketAllocator())
      : BucketManagerCnf<T>::BucketManagerCnf(occM, cache, mdStore,
                                              sizeFirstPage, sizeAdditionalPage,
                                              bucketAllocator),
        m_inConstruction(occM) {
    m_mapVar.resize(nbVarCnf + 1, 0);
    m_markIdx.resize(nbClauseCnf, -1);
    m_offsetClauses = new unsigned[nbClauseCnf];
  } // BucketManagerCnfCl

  /**
     Destructor.
   */
  ~BucketManagerCnfCl() { delete[] m_offsetClauses; } // destructor

  /**
   * Get an index to store the distribution information.
   *
   * @param[out] inConstruction, place where we store the bucket in
   * construction.
   *
   * \return the index of a reserved bucket.
   */
  inline int getIdxBucketSortInfo(BucketInConstruction &inConstruction) {
    int ret = m_unusedBucket;

    if (m_unusedBucket == -1) {
      ret = m_vecBucketSortInfo.size();
      m_vecBucketSortInfo.emplace_back(
          BucketSortInfo(inConstruction.nbClauseInDistrib));
    } else
      m_unusedBucket = -1;

    return ret;
  } // getIdxBucketSortInfo

  /**
   * Push sorted, use the natural order.
   */
  inline void pushSorted(unsigned *tab, unsigned pos, unsigned val) {
    tab[pos] = val;
    for (unsigned i = pos; i > 0; i--)
      if (tab[i] < tab[i - 1])
        std::swap(tab[i], tab[i - 1]);
      else
        break;
  } // pushSorted

  /**
     It is used in order to construct a sorted residual formula.

     @param[in] l, we considere the clause containing l
     @param[out] inConstruction, place where we store the bucket in
     construction.
  */
  void createDistribWrTLit(const Lit &l, BucketInConstruction &inConstruction) {
    unsigned currentPos = inConstruction.sizeDistrib; // where we put l.
    inConstruction.sizeDistrib += 2; // save memory for l and the size.

    // associate a bucket to the literal.
    unsigned counter = 0, nbElt = 0;
    unsigned *tab = &inConstruction.distrib[inConstruction.sizeDistrib];
    int ownBucket = getIdxBucketSortInfo(inConstruction);

    // visit each clause
    m_idInVecBucket.resize(0);
    unsigned nextBucket = m_vecBucketSortInfo.size();

    IteratorIdxClause listIndex = specManager.getVecIdxClause(l, modeStore);
    for (int *ptr = listIndex.start; ptr != listIndex.end; ptr++) {
      int idx = *ptr;
      if (!isKeptClause(idx))
        continue;

      assert((unsigned)idx < m_markIdx.size());
      if (m_markIdx[idx] == -1) {
        inConstruction.sizeClauses[idx] = 1;
        m_mustUnMark.push_back(idx);
        m_markIdx[idx] = ownBucket;
        pushSorted(tab, nbElt++, inConstruction.nbClauseInDistrib + counter);
        counter++;
      } else {
        inConstruction.sizeClauses[idx]++;
        BucketSortInfo &b = m_vecBucketSortInfo[m_markIdx[idx]];
        if (!b.counter) {
          assert(nextBucket ==
                 m_vecBucketSortInfo.size() + m_idInVecBucket.size());
          b.redirected = nextBucket++;
          m_idInVecBucket.push_back(m_markIdx[idx]);
        }
        m_markIdx[idx] = b.redirected;
        pushSorted(tab, nbElt++, b.start + b.counter);
        b.counter++;
      }
    }

    inConstruction.sizeDistrib += nbElt;
    assert(inConstruction.sizeDistrib < inConstruction.capacityDistrib);

    m_vecBucketSortInfo.resize(m_vecBucketSortInfo.size() +
                               m_idInVecBucket.size());
    for (auto &bid : m_idInVecBucket) {
      BucketSortInfo &b = m_vecBucketSortInfo[bid];
      assert(b.counter);

      // we split out the bucket.
      m_vecBucketSortInfo[b.redirected].reset(b.start, b.start + b.counter);
      b.start += b.counter;
      b.counter = 0;
    }

    if (!counter)
      m_unusedBucket = ownBucket;
    else {
      m_vecBucketSortInfo[ownBucket].reset(inConstruction.nbClauseInDistrib,
                                           inConstruction.nbClauseInDistrib +
                                               counter);
      inConstruction.nbClauseInDistrib += counter;
    }

    if (currentPos == inConstruction.sizeDistrib - 2)
      inConstruction.sizeDistrib -= 2;
    else {
      inConstruction.distrib[currentPos] = l.intern();
      inConstruction.distrib[currentPos + 1] =
          inConstruction.sizeDistrib - currentPos - 2;
    }
  } // createDistribWrTLit

  /**
     Collect the clause distribution. The result is stored in distrib.

     @param[in] component, the set of variables we consider.
     @param[out] inConstruction, place where we store the bucket in
     construction.

     \return the number of elements we have in the distribution once the
     redundant clauses have been removed.
  */
  inline unsigned collectDistrib(std::vector<Var> &component,
                                 BucketInConstruction &inConstruction) {
    // sort the set of clauses
    for (auto &v : component) {
      if (specManager.varIsAssigned(v))
        continue;
      createDistribWrTLit(Lit::makeLitFalse(v), inConstruction);
      createDistribWrTLit(Lit::makeLitTrue(v), inConstruction);
    }

    // mark the clause we do not keep.
    unsigned realSizeDistrib = inConstruction.sizeDistrib;
    for (auto &idx : m_mustUnMark) {
      BucketSortInfo &b = m_vecBucketSortInfo[m_markIdx[idx]];
      m_markIdx[idx] = -1;
      inConstruction.shiftedSizeClause[b.start] =
          inConstruction.sizeClauses[idx];
      if (b.end != b.start + 1) {
        realSizeDistrib -=
            (b.end - b.start - 1) * specManager.getCurrentSize(idx);
        for (unsigned j = b.start + 1; j < b.end; j++)
          inConstruction.markedAsRedundant[j] = true;
        b.end = b.start + 1;
      }
    }
    m_mustUnMark.resize(0);

    // shift the clauses indices if requiered.
    unsigned index = 0;
    for (unsigned i = 0; i < inConstruction.nbClauseInDistrib; i++) {
      if (!inConstruction.markedAsRedundant[i]) {
        inConstruction.distribDiffSize[inConstruction.shiftedSizeClause[i]]++;
        inConstruction.shiftedSizeClause[index] =
            inConstruction.shiftedSizeClause[i];
        inConstruction.shiftedIndexClause[i] = index++;
      } else
        inConstruction.shiftedIndexClause[i] = inConstruction.sizeDistrib;
      inConstruction.markedAsRedundant[i] = false;
    }
    inConstruction.nbClauseInDistrib = index; // resize
    return realSizeDistrib;
  } // collectDistrib

  /**
     Prepare the data to store a new bucket.

     @param[out] inConstruction, place where we store the bucket in
     construction.
   */
  inline void initSortBucket(BucketInConstruction &inConstruction) {
    inConstruction.reinit();
    m_unusedBucket = -1;
    m_vecBucketSortInfo.resize(0);
  } // initSortBucket

  /**
   * @brief Display the bucket in construction (for debugging purpose).
   *
   * @param v
   * @param out
   */
  inline void showListBucketSort(std::vector<BucketSortInfo> &v,
                                 std::ostream &out) {
    out << "size = " << v.size() << "\n";
    for (auto &e : v)
      out << "[" << e.start << " " << e.end << " " << e.counter << " "
          << e.redirected << "]\n";
  } // showListBucketSort

  /**
   * @brief Search for the number of bytes needed to store the different element
   * of the bucket.
   *
   * @param component is the set of variables.
   * @param inConstruction is the bucket that has been constructed.
   * @param nBda
   * @param nbD
   * @param nbEltData
   * @param nbEltDist
   * @return an AllocSizeInfo structure with the requiered information.
   */
  inline AllocSizeInfo computeNeededBytes(std::vector<Var> &component,
                                          BucketInConstruction &inConstruction,
                                          unsigned nBda, unsigned nbD,
                                          unsigned nbEltData,
                                          unsigned nbEltDist) {
    AllocSizeInfo ret;

    // info about the variables.
    ret.config = 0;
    ret.nbBitEltVar = nbBitUnsigned(component.back());
    ret.nbByteStoreVar = 1 + (((ret.nbBitEltVar * component.size()) - 1) >> 3);
    unsigned nbByteModeArray = 1 + ((component.back() - 1) >> 3);
    if (nbByteModeArray < ret.nbByteStoreVar) {
      ret.config |= 1;
      ret.nbByteStoreVar = nbByteModeArray;
      ret.nbBitEltVar = 0;
    }

    // info about the distribution.
    unsigned maxSizeCl = 0, maxLenghtDistrib = 0, cptDistrib = 0;
    for (unsigned i = 0; i <= inConstruction.maxSizeClause; i++) {
      if (!inConstruction.distribDiffSize[i])
        continue;

      maxSizeCl = i;
      cptDistrib++;
      if (inConstruction.distribDiffSize[i] > maxLenghtDistrib)
        maxLenghtDistrib = inConstruction.distribDiffSize[i];
    }
    if (cptDistrib)
      ret.initDistrib(maxLenghtDistrib, maxSizeCl, cptDistrib);

    std::cout << ret.nbByteStoreDistrib << "\n";

    ret.totalByte =
        ret.nbByteStoreVar + (nBda * nbEltData) + ret.nbByteStoreDistrib;
    return ret;
  } // computeNeededBytes

  /**
   * @brief
   *
   * @param p
   * @param val
   * @param nbBit
   * @param remainingBit
   * @return char*
   */
  inline char *addElementInData(char *p, unsigned val, unsigned nbBit,
                                unsigned &remainingBit) {
    if (!remainingBit) {
      remainingBit = 8;
      p++;
    }

    while (nbBit >= remainingBit) {
      *p <<= remainingBit;
      *p |= val & ((1 << remainingBit) - 1);
      val >>= remainingBit;
      nbBit -= remainingBit;
      remainingBit = 8;
      p++;
    }

    // the remaining bits.
    if (nbBit) {
      *p <<= nbBit;
      *p |= val;
      remainingBit -= nbBit;
      assert(remainingBit);
    }

    return p;
  } // addElementInData

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
  char *storeVariables(AllocSizeInfo &info, char *data,
                       std::vector<Var> &component) {
    char *p = data;

    // init with zero.
    for (unsigned i = 0; i < info.nbByteStoreVar; i++)
      p[i] = 0;

    if (info.config & 1) {
      for (auto v : component)
        p[v >> 3] |= ((uint8_t)1) << (v & 7);
    } else {
      unsigned remaining = 8, current = 0;
      for (auto v : component) {
        assert(v <= component.back());

        current = info.nbBitEltVar;
        while (current >= remaining) {
          *p <<= remaining;
          *p |= v & ((1 << remaining) - 1);
          v >>= remaining;
          current -= remaining;
          remaining = 8;
          p++;
        }

        // the remaining bits.
        if (current) {
          *p <<= current;
          *p |= v;
          v >>= current;
          remaining -= current;
          assert(remaining);
        }

        assert(!v);
      }
    }

    return &data[info.nbByteStoreVar];
  } // storeVariables

  /**
   Store the variables respecting the information of size concerning the type
   T to encode each elements and returns the pointer just after the end of the
   data.

   @param[in] data, the place where we store the information.
   @param[out] inConstruction, place where we store the bucket in
   construction.
 */
  char *storeDistribInfo(AllocSizeInfo &info, char *data,
                         BucketInConstruction &inConstruction) {
    char *p = data;
    unsigned remaining = 8, current = 0;
    for (unsigned i = 0; i < info.nbByteStoreDistrib; i++)
      p[i] = 0;

    for (unsigned i = 0; i <= inConstruction.maxSizeClause; i++) {
      if (!inConstruction.distribDiffSize[i])
        continue;

      // we store the size of the current block of clauses.
      unsigned val = i;
      current = info.nbBitEltDistribSizeCl;
      while (current >= remaining) {
        *p <<= remaining;
        *p |= val & ((1 << remaining) - 1);
        val >>= remaining;
        remaining = 8;
        p++;
      }

      if (current) {
        *p <<= current;
        *p |= val;
        val >>= current;
        remaining -= current;
        assert(remaining);
      }

      // the number of clauses of size i.
      val = inConstruction.distribDiffSize[i];
      current = info.nbBitEltDistribNbCl;
      while (current >= remaining) {
        *p <<= remaining;
        *p |= val & ((1 << remaining) - 1);
        val >>= remaining;
        remaining = 8;
        p++;
      }

      if (current) {
        *p <<= current;
        *p |= val;
        val >>= current;
        remaining -= current;
        assert(remaining);
      }
    }

    return &data[info.nbByteStoreDistrib];
  } // storeDistribInfo

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
  template <typename U>
  void *storeClauses(void *data, std::vector<Var> &component,
                     BucketInConstruction &inConstruction) {
    // map the variable to another index regarding their position in
    // component.
    for (unsigned i = 0; i < component.size(); i++)
      m_mapVar[component[i]] = i;

    // get information about the starting offset for the different clause
    // size.
    unsigned offSet = 0;
    unsigned memoryPlaceWrtSizeClause[m_maxSizeClause + 1];
    for (unsigned i = 0; i <= m_maxSizeClause; i++) {
      memoryPlaceWrtSizeClause[i] = offSet;
      offSet += inConstruction.distribDiffSize[i] * i;
    }

    // allocate an offset for each clauses.
    for (unsigned i = 0; i < inConstruction.nbClauseInDistrib; i++) {
      unsigned szClause = inConstruction.shiftedSizeClause[i];
      if (!szClause)
        continue;

      m_offsetClauses[i] = memoryPlaceWrtSizeClause[szClause];
      memoryPlaceWrtSizeClause[szClause] += szClause;
      inConstruction.shiftedSizeClause[i] = 0;
    }

    // we store the data.
    U *p = static_cast<U *>(data);
    unsigned i = 0;
    while (i < inConstruction.sizeDistrib) {
      unsigned lit = inConstruction.distrib[i++];

      U l = static_cast<U>((m_mapVar[lit >> 1] << 1) | (lit & 1));
      unsigned szLitList = inConstruction.distrib[i++];

      while (szLitList) {
        szLitList--;

        unsigned idx =
            inConstruction.shiftedIndexClause[inConstruction.distrib[i++]];
        if (idx >= inConstruction.nbClauseInDistrib)
          continue;
        p[m_offsetClauses[idx]] = l;
        m_offsetClauses[idx]++;
      }
    }

    p += offSet;
    return p;
  } // storeClauses

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
  inline void getInfoDistributionSize(unsigned &maxNbSizeClause,
                                      unsigned &largestSizeClause,
                                      unsigned &nbDiffClauseSize,
                                      unsigned &nbLit,
                                      BucketInConstruction &inConstruction) {
    largestSizeClause = 0;
    maxNbSizeClause = 0;
    nbDiffClauseSize = 0;
    for (unsigned i = 0; i <= m_maxSizeClause; i++)
      if (inConstruction.distribDiffSize[i]) {
        largestSizeClause = i;
        if (maxNbSizeClause < inConstruction.distribDiffSize[i])
          maxNbSizeClause = inConstruction.distribDiffSize[i];
        nbDiffClauseSize++;
        nbLit += inConstruction.distribDiffSize[i] * i;
      }
  } // getInfoDistributionSize

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables.
     @param[out] tmpFormula, the place where is stored the formula.
     @param[out] szTmpFormula, to collect the size of the stored formula.
  */
  inline void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) {
    initSortBucket(m_inConstruction);
    collectDistrib(component, m_inConstruction); // built the sorted formula

    // get information about the clause distribution
    unsigned nbLit = 0, nbVar = component.size(), maxNbSizeClause,
             nbDiffClauseSize, largestSizeClause;
    getInfoDistributionSize(maxNbSizeClause, largestSizeClause,
                            nbDiffClauseSize, nbLit, m_inConstruction);

    unsigned nbODistrib =
        this->nbOctetToEncodeInt(std::max(maxNbSizeClause, largestSizeClause));
    unsigned nbOLit = this->nbOctetToEncodeInt(nbVar << 1);

    // ask for memory
    AllocSizeInfo sizeInfo =
        computeNeededBytes(component, m_inConstruction, nbOLit, nbODistrib,
                           nbLit, nbDiffClauseSize);
    char *data = m_bucketAllocator->getArray(sizeInfo.totalByte);
    void *p = static_cast<void *>(
        &data[sizeInfo.nbByteStoreVar + sizeInfo.nbByteStoreDistrib]);

    storeVariables(sizeInfo, data, component);

    if (!m_inConstruction.nbClauseInDistrib)
      goto fillTheBucket;

    storeDistribInfo(sizeInfo, &data[sizeInfo.nbByteStoreVar],
                     m_inConstruction);

    // store the clauses.
    switch (nbOLit) {
    case 1:
      storeClauses<uint8_t>(p, component, m_inConstruction);
      break;
    case 2:
      storeClauses<uint16_t>(p, component, m_inConstruction);
      break;
    case 4:
      storeClauses<uint32_t>(p, component, m_inConstruction);
      break;
    default:
      throw(BucketException("Bad number of bytes", __FILE__, __LINE__));
    }

  fillTheBucket:
    // put the information into the bucket
    DataInfoCnfCl di(sizeInfo.totalByte, nbVar, nbLit, nbDiffClauseSize,
                     sizeInfo.nbBitEltVar, nbOLit, nbODistrib);

    assert(di.szData() == sizeInfo.totalByte);
    b.set(data, di);
  } // storeFormula
};
} // namespace d4
