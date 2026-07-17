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

#include "FormulaStoreCnfSym.hpp"

namespace d4 {

FormulaStoreCnfSym::FormulaStoreCnfSym(CnfManager& occM, ModeStore mdStore)
    : FormulaStoreCnf(occM, mdStore), m_inConstruction(occM) {
  this->m_mapVar.resize(this->m_nbVarCnf + 1, 0);
  this->m_markIdx.resize(this->m_nbClauseCnf, -1);
  this->m_offsetClauses = new unsigned[this->m_nbClauseCnf];

  m_storeNbClause.resize((1 + this->m_nbVarCnf) << 1, 0);
  m_litDegree.resize((1 + this->m_nbVarCnf) << 1, 0);

  m_stampDegreeVector.resize((1 + this->m_nbVarCnf), 0);
  m_stampDegreeIndex = 0;
  m_memoryPlaceWrtSizeClause = new unsigned[this->m_maxSizeClause + 1];
}

FormulaStoreCnfSym::~FormulaStoreCnfSym() {
  delete[] m_offsetClauses;
  delete[] m_memoryPlaceWrtSizeClause;
}

int FormulaStoreCnfSym::getIdxBucketSortInfo(
    BucketInConstruction& inConstruction) {
  int ret = m_unusedBucket;

  if (m_unusedBucket == -1) {
    ret = m_vecBucketSortInfo.size();
    m_vecBucketSortInfo.emplace_back(
        BucketSortInfo(inConstruction.nbClauseInDistrib));
  } else
    m_unusedBucket = -1;

  return ret;
}

void FormulaStoreCnfSym::pushSorted(unsigned* tab, unsigned pos, unsigned val) {
  tab[pos] = val;
  for (unsigned i = pos; i > 0; i--)
    if (tab[i] < tab[i - 1])
      std::swap(tab[i], tab[i - 1]);
    else
      break;
}

void FormulaStoreCnfSym::createDistribWrTLit(
    const Lit& l, BucketInConstruction& inConstruction, const Lit repLit) {
  unsigned currentPos = inConstruction.sizeDistrib;
  inConstruction.sizeDistrib += 2;

  unsigned counter = 0, nbElt = 0;
  unsigned* tab = &inConstruction.distrib[inConstruction.sizeDistrib];
  int ownBucket = getIdxBucketSortInfo(inConstruction);

  m_idInVecBucket.resize(0);
  unsigned nextBucket = m_vecBucketSortInfo.size();

  IteratorIdxClause listIndex = this->m_specManager.getVecIdxClause(l);
  for (int* ptr = listIndex.start; ptr != listIndex.end; ptr++) {
    int idx = *ptr;
    assert((unsigned)idx < m_markIdx.size());
    if (m_markIdx[idx] == -1) {
      inConstruction.sizeClauses[idx] = 1;
      m_mustUnMark.push_back(idx);
      m_markIdx[idx] = ownBucket;
      pushSorted(tab, nbElt++, inConstruction.nbClauseInDistrib + counter);
      counter++;
    } else {
      inConstruction.sizeClauses[idx]++;
      BucketSortInfo& b = m_vecBucketSortInfo[m_markIdx[idx]];
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
  for (auto& bid : m_idInVecBucket) {
    BucketSortInfo& b = m_vecBucketSortInfo[bid];
    assert(b.counter);
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

  if (currentPos == inConstruction.sizeDistrib - 2)
    inConstruction.sizeDistrib -= 2;
  else {
    inConstruction.distrib[currentPos] = l.intern();
    inConstruction.distrib[currentPos + 1] =
        inConstruction.sizeDistrib - currentPos - 2;
  }
}

unsigned FormulaStoreCnfSym::collectDistrib(
    std::vector<Lit>& orderedLit, BucketInConstruction& inConstruction) {
  for (auto& l : orderedLit) {
    if (this->m_specManager.varIsAssigned(l.var())) continue;
    createDistribWrTLit(l, inConstruction, l);
    createDistribWrTLit(~l, inConstruction, ~l);
  }

  unsigned realSizeDistrib = inConstruction.sizeDistrib;
  for (auto& idx : m_mustUnMark) {
    BucketSortInfo& b = m_vecBucketSortInfo[m_markIdx[idx]];
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

  unsigned index = 0;
  for (unsigned i = 0; i < inConstruction.nbClauseInDistrib; i++) {
    if (!inConstruction.markedAsRedundant[i]) {
      inConstruction.addClauseSize(inConstruction.shiftedSizeClause[i]);
      inConstruction.shiftedSizeClause[index] =
          inConstruction.shiftedSizeClause[i];
      inConstruction.shiftedIndexClause[i] = index++;
    } else
      inConstruction.shiftedIndexClause[i] = inConstruction.sizeDistrib;
    inConstruction.markedAsRedundant[i] = false;
  }
  inConstruction.nbClauseInDistrib = index;
  inConstruction.sortPresentSizes();

  return realSizeDistrib;
}

void FormulaStoreCnfSym::initSortBucket(BucketInConstruction& inConstruction) {
  inConstruction.reinit();
  m_unusedBucket = -1;
  m_vecBucketSortInfo.resize(0);
}

void FormulaStoreCnfSym::showListBucketSort(std::vector<BucketSortInfo>& v,
                                            std::ostream& out) {
  out << "size = " << v.size() << "\n";
  for (auto& e : v)
    out << "[" << e.start << " " << e.end << " " << e.counter << " "
        << e.redirected << "]";
  out << "\n";
}

unsigned FormulaStoreCnfSym::computeNeededBytes(unsigned nBda, unsigned nbD,
                                                unsigned nbEltData,
                                                unsigned nbEltDist) {
  return (nBda * nbEltData) + (nbD * (nbEltDist << 1));
}

template <typename U>
void* FormulaStoreCnfSym::storeVariables(void* data,
                                         std::vector<Var>& component) {
  U* p = static_cast<U*>(data);
  for (auto& v : component) {
    *p = static_cast<U>(v);
    p++;
  }
  return p;
}

template <typename U>
void* FormulaStoreCnfSym::storeDistribInfo(void* data,
                                           BucketInConstruction& inConstruction) {
  U* p = static_cast<U*>(data);
  for (unsigned sz : inConstruction.presentSizes) {
    *p = static_cast<U>(sz);
    p++;
    *p = static_cast<U>(inConstruction.distribDiffSize[sz]);
    p++;
  }
  return p;
}

template <typename U>
void* FormulaStoreCnfSym::storeClauses(void* data,
                                       std::span<const Var> component,
                                       BucketInConstruction& inConstruction) {
  for (unsigned i = 0; i < component.size(); i++) m_mapVar[component[i]] = i;

  unsigned offSet = 0;
  for (unsigned sz : inConstruction.presentSizes) {
    m_memoryPlaceWrtSizeClause[sz] = offSet;
    offSet += inConstruction.distribDiffSize[sz] * sz;
  }

  for (unsigned i = 0; i < inConstruction.nbClauseInDistrib; i++) {
    unsigned szClause = inConstruction.shiftedSizeClause[i];
    if (!szClause) continue;
    m_offsetClauses[i] = m_memoryPlaceWrtSizeClause[szClause];
    m_memoryPlaceWrtSizeClause[szClause] += szClause;
    inConstruction.shiftedSizeClause[i] = 0;
  }

  U* p = static_cast<U*>(data);
  unsigned i = 0;
  while (i < inConstruction.sizeDistrib) {
    unsigned lit = inConstruction.distrib[i++];
    U l = static_cast<U>((m_mapVar[lit >> 1] << 1) | (lit & 1));
    unsigned szLitList = inConstruction.distrib[i++];

    while (szLitList) {
      szLitList--;
      unsigned idx =
          inConstruction.shiftedIndexClause[inConstruction.distrib[i++]];
      if (idx >= inConstruction.nbClauseInDistrib) continue;
      p[m_offsetClauses[idx]] = l;
      m_offsetClauses[idx]++;
    }
  }

  p += offSet;
  return p;
}

void FormulaStoreCnfSym::getInfoDistributionSize(
    unsigned& maxNbSizeClause, unsigned& largestSizeClause,
    unsigned& nbDiffClauseSize, unsigned& nbLit,
    BucketInConstruction& inConstruction) {
  largestSizeClause = 0;
  maxNbSizeClause = 0;
  nbDiffClauseSize = inConstruction.presentSizes.size();
  for (unsigned sz : inConstruction.presentSizes) {
    largestSizeClause = sz;
    if (maxNbSizeClause < inConstruction.distribDiffSize[sz])
      maxNbSizeClause = inConstruction.distribDiffSize[sz];
    nbLit += inConstruction.distribDiffSize[sz] * sz;
  }
}

#define MORE_SYM 0

void FormulaStoreCnfSym::varToSortedLiterals(std::span<const Var> component,
                                             std::vector<Lit>& orderedLits) {
  orderedLits.clear();
  orderedLits.reserve(component.size());
  for (auto v : component) {
    Lit l = Lit::makeLitTrue(v);
    orderedLits.push_back(l);
    m_storeNbClause[l.intern()] = m_specManager.getNbClause(l);
    m_storeNbClause[(~l).intern()] = m_specManager.getNbClause(~l);
  }

#if MORE_SYM
  for (auto v : component) {
    Lit l = Lit::makeLitTrue(v);
    for (auto m : {l, ~l}) {
      m_stampDegreeIndex++;
      m_stampDegreeVector[m.var()] = m_stampDegreeIndex;
      IteratorIdxClause listIndex = this->m_specManager.getVecIdxClause(m);
      for (int* ptr = listIndex.start; ptr != listIndex.end; ptr++) {
        for (auto k : this->m_specManager.getClause(*ptr))
          if (m_stampDegreeVector[k.var()] != m_stampDegreeIndex) {
            m_stampDegreeVector[k.var()] = m_stampDegreeIndex;
            m_litDegree[m.intern()]++;
          }
      }
    }
  }
#endif

  for (auto& l : orderedLits)
    if (m_storeNbClause[l.intern()] > m_storeNbClause[(~l).intern()]) l = ~l;

  std::sort(
      orderedLits.begin(), orderedLits.end(), [&](const Lit l1, const Lit l2) {
        if (m_storeNbClause[l1.intern()] != m_storeNbClause[l2.intern()])
          return m_storeNbClause[l1.intern()] < m_storeNbClause[l2.intern()];
        if (m_storeNbClause[(~l1).intern()] != m_storeNbClause[(~l2).intern()])
          return m_storeNbClause[(~l1).intern()] <
                 m_storeNbClause[(~l2).intern()];
#if MORE_SYM
        if (m_litDegree[l1.intern()] != m_litDegree[l2.intern()])
          return m_litDegree[l1.intern()] < m_litDegree[l2.intern()];
        if (m_litDegree[(~l1).intern()] != m_litDegree[(~l2).intern()])
          return m_litDegree[(~l1).intern()] < m_litDegree[(~l2).intern()];
        unsigned countL1 =
            m_litDegree[l1.intern()] + m_litDegree[(~l1).intern()];
        unsigned countL2 =
            m_litDegree[l2.intern()] + m_litDegree[(~l2).intern()];
        if (countL1 != countL2) return countL1 < countL2;
#endif
        return l1.var() < l2.var();
      });
}

void FormulaStoreCnfSym::storeFormula(std::span<const Var> component,
                                      DataBucket& b, BucketAllocator& alloc) {
  std::vector<Lit> literalOrdered;
  varToSortedLiterals(component, literalOrdered);

  initSortBucket(m_inConstruction);
  collectDistrib(literalOrdered, m_inConstruction);

  unsigned nbLit = 0, nbVar = component.size(), maxNbSizeClause,
           nbDiffClauseSize, largestSizeClause;
  getInfoDistributionSize(maxNbSizeClause, largestSizeClause, nbDiffClauseSize,
                          nbLit, m_inConstruction);

  unsigned nbODistrib =
      this->nbOctetToEncodeInt(std::max(maxNbSizeClause, largestSizeClause));
  unsigned nbOLit = this->nbOctetToEncodeInt(nbVar << 1);

  unsigned szData =
      computeNeededBytes(nbOLit, nbODistrib, nbLit, nbDiffClauseSize);
  char* data = alloc.getArray(szData);
  void* p = data;

  switch (nbODistrib) {
    case 1:
      p = storeDistribInfo<uint8_t>(p, m_inConstruction);
      break;
    case 2:
      p = storeDistribInfo<uint16_t>(p, m_inConstruction);
      break;
    case 4:
      p = storeDistribInfo<uint32_t>(p, m_inConstruction);
      break;
    default:
      throw(BucketException("Bad number of bytes", __FILE__, __LINE__));
  }
  assert(static_cast<char*>(p) == &data[nbODistrib * (nbDiffClauseSize << 1)]);

  switch (nbOLit) {
    case 1:
      p = storeClauses<uint8_t>(p, component, m_inConstruction);
      break;
    case 2:
      p = storeClauses<uint16_t>(p, component, m_inConstruction);
      break;
    case 4:
      p = storeClauses<uint32_t>(p, component, m_inConstruction);
      break;
    default:
      throw(BucketException("Bad number of bytes", __FILE__, __LINE__));
  }
  assert(static_cast<char*>(p) == &data[szData]);

  DataInfo di(szData, component.size(), 0,
              this->nbBitUnsigned(2 + (component.size() << 1)));
  assert(di.szData() == szData);
  b.set(data, di);
}

}  // namespace d4
