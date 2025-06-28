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

#include "BucketManagerCnfCl.hpp"

namespace d4 {

/**
 * @brief BucketManagerCnfCl::BucketManagerCnfCl implementation.
 */
BucketManagerCnfCl::BucketManagerCnfCl(CnfManager &occM, ModeStore mdStore,
                                       unsigned long sizeFirstPage,
                                       unsigned long sizeAdditionalPage,
                                       BucketAllocator *bucketAllocator)
    : BucketManagerCnf::BucketManagerCnf(occM, mdStore, sizeFirstPage,
                                         sizeAdditionalPage, bucketAllocator),
      m_inConstruction(occM) {
  m_mapVar.resize(this->m_nbVarCnf + 1, 0);
  m_markIdx.resize(this->m_nbClauseCnf, -1);
  m_memoryPosWrtClauseSize = new unsigned[occM.getMaxSizeClause() + 1];
  m_offsetClauses = new unsigned[this->m_nbClauseCnf + 1];
}  // BucketManagerCnfCl

/**
 * @brief BucketManagerCnfCl::~BucketManagerCnfCl implementation.
 */
BucketManagerCnfCl::~BucketManagerCnfCl() {
  delete[] m_memoryPosWrtClauseSize;
  delete[] m_offsetClauses;
}  // destructor

/**
 * @brief BucketManagerCnfCl::getIdxBucketSortInfo implementation.
 */
int BucketManagerCnfCl::getIdxBucketSortInfo(
    BucketInConstruction &inConstruction) {
  int ret = m_unusedBucket;

  if (m_unusedBucket == -1) {
    ret = m_vecBucketSortInfo.size();
    m_vecBucketSortInfo.emplace_back(
        BucketSortInfo(inConstruction.nbClauseInDistrib));
  } else
    m_unusedBucket = -1;

  return ret;
}  // getIdxBucketSortInfo

/**
 * @brief BucketManagerCnfCl::pushSorted implementation.
 */
void BucketManagerCnfCl::pushSorted(unsigned *tab, unsigned pos, unsigned val) {
  tab[pos] = val;
  for (unsigned i = pos; i > 0; i--)
    if (tab[i] < tab[i - 1])
      std::swap(tab[i], tab[i - 1]);
    else
      break;
}  // pushSorted

/**
 * @brief BucketManagerCnfCl::createDistribWrTLit implementation.
 */
void BucketManagerCnfCl::createDistribWrTLit(
    const Lit &l, BucketInConstruction &inConstruction) {
  if (this->canSkipLit(l)) return;

  unsigned currentPos = inConstruction.sizeDistrib;  // where we put l.
  inConstruction.sizeDistrib += 2;  // save memory for l and the size.

  // associate a bucket to the literal.
  unsigned counter = 0, nbElt = 0;
  unsigned *tab = &inConstruction.distrib[inConstruction.sizeDistrib];
  int ownBucket = getIdxBucketSortInfo(inConstruction);

  // visit each clause
  m_idInVecBucket.resize(0);
  unsigned nextBucket = m_vecBucketSortInfo.size();

  IteratorIdxClause listIndex =
      this->m_specManager.getVecIdxClause(l, this->m_modeStore);
  for (int *ptr = listIndex.start; ptr != listIndex.end; ptr++) {
    int idx = *ptr;
    if (!this->isKeptClause(idx)) continue;

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
    m_vecBucketSortInfo[ownBucket].reset(
        inConstruction.nbClauseInDistrib,
        inConstruction.nbClauseInDistrib + counter);
    inConstruction.nbClauseInDistrib += counter;
  }

  if (currentPos == inConstruction.sizeDistrib - 2) {
    inConstruction.sizeDistrib -= 2;
  } else {
    inConstruction.distrib[currentPos] = l.intern();
    inConstruction.distrib[currentPos + 1] =
        inConstruction.sizeDistrib - currentPos - 2;
  }
}  // createDistribWrTLit

/**
 * @brief BucketManagerCnfCl::collectDistrib implementation.
 */
unsigned BucketManagerCnfCl::collectDistrib(
    std::vector<Var> &component, BucketInConstruction &inConstruction) {
  // sort the set of clauses
  for (auto &v : component) {
    if (this->m_specManager.varIsAssigned(v)) continue;
    createDistribWrTLit(Lit::makeLitFalse(v), inConstruction);
    createDistribWrTLit(Lit::makeLitTrue(v), inConstruction);
  }

  // mark the clause we do not keep.
  unsigned realSizeDistrib = inConstruction.sizeDistrib;
  for (auto &idx : m_mustUnMark) {
    BucketSortInfo &b = m_vecBucketSortInfo[m_markIdx[idx]];
    m_markIdx[idx] = -1;
    inConstruction.shiftedSizeClause[b.start] = inConstruction.sizeClauses[idx];
    if (b.end != b.start + 1) {
      realSizeDistrib -=
          (b.end - b.start - 1) * this->m_specManager.getCurrentSize(idx);
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
  inConstruction.nbClauseInDistrib = index;  // resize
  return realSizeDistrib;
}  // collectDistrib

/**
 * @brief BucketManagerCnfCl::initSortBucket implementation.
 */
void BucketManagerCnfCl::initSortBucket(BucketInConstruction &inConstruction) {
  inConstruction.reinit();
  m_unusedBucket = -1;
  m_vecBucketSortInfo.resize(0);
}  // initSortBucket

/**
 * @brief BucketManagerCnfCl::showListBucketSort implementation.
 */
void BucketManagerCnfCl::showListBucketSort(std::vector<BucketSortInfo> &v,
                                            std::ostream &out) {
  out << "size = " << v.size() << "\n";
  for (auto &e : v)
    out << "[" << e.start << " " << e.end << " " << e.counter << " "
        << e.redirected << "]\n";
}  // showListBucketSort

/**
 * @brief BucketManagerCnfCl::computeNeededBytes implementation.
 */
AllocSizeInfo BucketManagerCnfCl::computeNeededBytes(
    std::vector<Var> &component, BucketInConstruction &inConstruction) {
  AllocSizeInfo ret;

  // info about the variables.
  ret.nbBitEltVar = BucketManager::nbBitUnsigned(component.back());
  ret.nbByteStoreVar = 1 + (((ret.nbBitEltVar * component.size()) - 1) >> 3);
  unsigned nbByteModeArray = 1 + ((component.back() - 1) >> 3);

  // check if we can use a bit representation of the variables.
  if (nbByteModeArray < ret.nbByteStoreVar) {
    ret.nbByteStoreVar = nbByteModeArray;
    ret.nbBitEltVar = 0;
  }

  ret.nbBitStoreLit = BucketManager::nbBitUnsigned(2 + (component.size() << 1));

  // info about the distribution.
  unsigned cptLitFormula = 0, cptDistrib = 0;
  for (unsigned i = 0; i <= inConstruction.maxSizeClause; i++) {
    if (!inConstruction.distribDiffSize[i]) continue;

    cptDistrib++;
    cptLitFormula += i * inConstruction.distribDiffSize[i];
  }

  ret.nbByteStoreFormula =
      (!cptDistrib)
          ? 0
          : 1 + ((ret.nbBitStoreLit * ((cptDistrib << 1) + cptLitFormula)) >>
                 3);

  ret.totalByte = ret.nbByteStoreVar + ret.nbByteStoreFormula;
  return ret;
}  // computeNeededBytes

/**
 * @brief BucketManagerCnfCl::addElementInData implementation.
 */
char *BucketManagerCnfCl::addElementInData(char *p, unsigned val,
                                           unsigned nbBit,
                                           unsigned &remainingBit) {
  if (!remainingBit) {
    remainingBit = 8;
    p++;
  }

  while (nbBit >= remainingBit) {
    *p |= val & ((1 << remainingBit) - 1);
    val >>= remainingBit;
    nbBit -= remainingBit;
    remainingBit = 8;
    p++;
  }

  // the remaining bits.
  if (nbBit) {
    *p |= val << (remainingBit - nbBit);
    remainingBit -= nbBit;
    assert(remainingBit);
  }

  return p;
}  // addElementInData

/**
 * @brief BucketManagerCnfCl::storeVariables implementation.
 */
char *BucketManagerCnfCl::storeVariables(AllocSizeInfo &info, char *data,
                                         std::vector<Var> &component) {
  // init the array.
  char *p = data;
  memset(p, 0, info.nbByteStoreVar);

  // fill the array.
  if (!info.nbBitEltVar) {
    for (auto v : component) p[v >> 3] |= ((uint8_t)1) << (v & 7);
  } else {
    unsigned remaining = 8;
    for (auto v : component) {
      assert(v <= component.back());
      p = addElementInData(p, v, info.nbBitEltVar, remaining);
    }
  }

  return &data[info.nbByteStoreVar];
}  // storeVariables

/**
 * @brief BucketManagerCnfCl::storeClauses implementation.
 */
char *BucketManagerCnfCl::storeClauses(AllocSizeInfo &info, char *data,
                                       std::vector<Var> &component,
                                       BucketInConstruction &inConstruction) {
  unsigned remaining = 8;
  char *p = data;
  memset(p, 0, info.nbByteStoreFormula);

  // map the variables to their position.
  for (unsigned i = 0; i < component.size(); i++) {
    m_mapVar[component[i]] = i + 1;
  }

  // store the different size of the distribution.
  for (unsigned i = 0; i <= inConstruction.maxSizeClause; i++) {
    if (!inConstruction.distribDiffSize[i]) continue;
    p = addElementInData(p, i, info.nbBitStoreLit, remaining);
  }

  // Prepare the offset list regarding p.
  // We also add a zero to separate ditrib from formula.
  unsigned offSet = (8 - remaining) + info.nbBitStoreLit;
  for (unsigned i = 0; i <= inConstruction.maxSizeClause; i++) {
    if (!inConstruction.distribDiffSize[i]) continue;
    m_memoryPosWrtClauseSize[i] = offSet;
    offSet += info.nbBitStoreLit +
              inConstruction.distribDiffSize[i] * i * info.nbBitStoreLit;
  }

  // allocate an offset for each clauses.
  for (unsigned i = 0; i < inConstruction.nbClauseInDistrib; i++) {
    unsigned &szClause = inConstruction.shiftedSizeClause[i];
    if (!szClause) continue;

    m_offsetClauses[i] = m_memoryPosWrtClauseSize[szClause];
    m_memoryPosWrtClauseSize[szClause] += szClause * info.nbBitStoreLit;
    szClause = 0;  // reinit shiftedSizeClause for the next run.
  }

  // store the formula.
  unsigned i = 0;
  while (i < inConstruction.sizeDistrib) {
    unsigned lit = inConstruction.distrib[i++];
    unsigned l = (m_mapVar[lit >> 1] << 1) | (lit & 1);
    unsigned szLitList = inConstruction.distrib[i++];

    while (szLitList) {
      szLitList--;

      unsigned idx =
          inConstruction.shiftedIndexClause[inConstruction.distrib[i++]];
      if (idx >= inConstruction.nbClauseInDistrib) continue;

      // compute the position in the array.
      unsigned &offSet = m_offsetClauses[idx];
      char *q = &p[offSet >> 3];  // divided by 8
      remaining = 8 - (offSet & 7);

      // add the element and move the offset for the next lit.
      addElementInData(q, l, info.nbBitStoreLit, remaining);
      offSet += info.nbBitStoreLit;
    }
  }

  return &data[info.nbByteStoreFormula];
}  // storeClauses

/**
 * @brief BucketManagerCnfCl::storeFormula implementation.
 */
void BucketManagerCnfCl::storeFormula(std::vector<Var> &component,
                                      DataBucket &b) {
  initSortBucket(m_inConstruction);
  collectDistrib(component, m_inConstruction);  // built the sorted formula

  // ask for memory
  AllocSizeInfo sizeInfo = computeNeededBytes(component, m_inConstruction);
  char *data = this->m_bucketAllocator->getArray(sizeInfo.totalByte);

  // store the information about the formula.
  storeVariables(sizeInfo, data, component);
  if (m_inConstruction.nbClauseInDistrib)
    storeClauses(sizeInfo, &data[sizeInfo.nbByteStoreVar], component,
                 m_inConstruction);
  // put the information into the bucket
  DataInfo di(sizeInfo.totalByte, component.size(), sizeInfo.nbBitEltVar,
              sizeInfo.nbBitStoreLit);
  b.set(data, di);
}  // storeFormula
}  // namespace d4