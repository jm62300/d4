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

#include "src/exceptions/BucketException.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "DataInfoCnfCl.hpp"
#include "BucketManagerCnf.hpp"

#define PRINT_DEBUG 0


namespace d4
{
class BucketSortInfo
{
 public:
  unsigned start;
  unsigned end;
  unsigned counter;
  unsigned redirected; // redirected only if counter > 0

  BucketSortInfo() : start(0), end(0), counter(0), redirected(0) {}
  BucketSortInfo(unsigned init) : start(init), end(init), counter(0), redirected(0) {}
  BucketSortInfo(unsigned s, unsigned e) : start(s), end(e), counter(0), redirected(0) {}

  inline void display(std::ostream &out)
  {
    out << start << " " << end << "\n";
  }

  inline void reset(unsigned s, unsigned e)
  {
    start = s;
    end = e;
    counter = redirected = 0;
  }
};


template<class T> class BucketManagerCnfCl : public BucketManagerCnf<T>
{
 private:
  std::vector<BucketSortInfo> m_vecBucketSortInfo;
  int m_unusedBucket;
  std::vector<unsigned long int> m_mapVar;
  
  std::vector<int> m_mustUnMark;
  std::vector<int> m_markIdx;
  std::vector<unsigned> m_idInVecBucket;
  unsigned m_lastSize;

  unsigned *m_distrib;
  unsigned *m_offsetClauses;
  unsigned *m_shiftedIndexClause;
  unsigned *m_shiftedSizeClause;
  bool *m_markedAsRedundant;
  unsigned *m_sizeClauses;
  unsigned *m_distribDiffSize;
  
  unsigned m_nbClauseInDistrib;
  unsigned m_sizeDistrib;
  unsigned m_capacityDistrib;
  

  // using: variables
  using BucketManagerCnf<T>::specManager;
  using BucketManagerCnf<T>::nbClauseCnf;
  using BucketManagerCnf<T>::nbVarCnf;
  using BucketManagerCnf<T>::m_maxSizeClause;
  using BucketManagerCnf<T>::m_idxClauses;

  // using: functions
  using BucketManagerCnf<T>::isKeptClause;
  using BucketManagerCnf<T>::collectIdActiveClauses;


 public:
  /**
     Function called in order to initialized variables before using
  */
  BucketManagerCnfCl(SpecManagerCnf &occM, int mdStore, unsigned sizePage) :
      BucketManagerCnf<T>::BucketManagerCnf(occM, mdStore, sizePage)
  {
    m_mapVar.resize(nbVarCnf + 1, 0);
    m_markIdx.resize(nbClauseCnf, -1);
    
    m_capacityDistrib =  3 * occM.getSumSizeClauses() + nbVarCnf;
    m_sizeDistrib = 0;
    m_nbClauseInDistrib = 0;


    m_offsetClauses = new unsigned[nbClauseCnf];
    m_shiftedIndexClause = new unsigned[nbClauseCnf];
    m_distrib = new unsigned[m_capacityDistrib];
    m_markedAsRedundant = new bool[nbClauseCnf];
    m_sizeClauses = new unsigned[nbClauseCnf];
    m_shiftedSizeClause = new unsigned[nbClauseCnf];
    m_distribDiffSize = new unsigned[m_maxSizeClause + 1];

    for(unsigned i = 0 ; i<nbClauseCnf ; i++) m_markedAsRedundant[i] = false;
  }// BucketManagerCnfCl


  /**
     Destructor.
   */
  ~BucketManagerCnfCl()
  {
    delete[] m_offsetClauses;
    delete[] m_shiftedSizeClause;
    delete[] m_distribDiffSize;
    delete[] m_shiftedIndexClause;
    delete[] m_distrib;
    delete[] m_markedAsRedundant;
    delete[] m_sizeClauses;
  } // destructor


  /**
     Get an index store the distribution information.

     \return the index of a reserved bucket.
   */
  inline int getIdxBucketSortInfo()
  {
    int ret = m_unusedBucket;
    
    if(m_unusedBucket == -1)
    {
      ret = m_vecBucketSortInfo.size();
      m_vecBucketSortInfo.emplace_back(BucketSortInfo(m_nbClauseInDistrib));
    } else m_unusedBucket = -1;

    return ret;
  } // getIdxBucketSortInfo


  /**
     Push sorted, use the natural order.
     
   */
  inline void pushSorted(unsigned *tab, unsigned pos, unsigned val)
  {
    tab[pos] = val;
    for(unsigned i = pos ; i > 0 ; i--)
      if(tab[i] < tab[i-1]) std::swap(tab[i], tab[i-1]);
      else break;
  } // pushSorted
  
  
  /**
     It is used in order to construct a sorted residual formula.

     @param[in] l, we considere the clause containing l
  */
  void createDistribWrTLit(const Lit &l)
  {
    unsigned currentPos = m_sizeDistrib; // the place where we put l.
    m_sizeDistrib += 2;                  // save memory for l and the size.
    
    // associate a bucket to the literal.
    unsigned counter = 0, nbElt = 0, *tab = &m_distrib[m_sizeDistrib];
    int ownBucket = getIdxBucketSortInfo();
    
    // visit each clause
    m_idInVecBucket.resize(0);
    unsigned nextBucket = m_vecBucketSortInfo.size();
    for(auto &idx : specManager.getVecIdxClause(l))
    {
      if(!isKeptClause(idx)) continue;      
      
      assert((unsigned) idx < m_markIdx.size());
      if(m_markIdx[idx] == -1)
      {
        m_sizeClauses[idx] = 1;
        m_mustUnMark.push_back(idx);
        m_markIdx[idx] = ownBucket;
        pushSorted(tab, nbElt++, m_nbClauseInDistrib + counter);
        counter++;        
      }else
      {
        m_sizeClauses[idx]++;
        BucketSortInfo &b = m_vecBucketSortInfo[m_markIdx[idx]];
        if(!b.counter)
        {
          assert(nextBucket == m_vecBucketSortInfo.size() + m_idInVecBucket.size());
          b.redirected = nextBucket++;
          m_idInVecBucket.push_back(m_markIdx[idx]);
        }
        m_markIdx[idx] = b.redirected;
        pushSorted(tab, nbElt++, b.start + b.counter);
        b.counter++;        
      }
    }
    
    m_sizeDistrib += nbElt;
    assert(m_sizeDistrib < m_capacityDistrib);
    
    m_vecBucketSortInfo.resize(m_vecBucketSortInfo.size() + m_idInVecBucket.size());
    for(auto &bid : m_idInVecBucket)
    {
      BucketSortInfo &b = m_vecBucketSortInfo[bid];
      assert(b.counter);
      
      // we split out the bucket.
      m_vecBucketSortInfo[b.redirected].reset(b.start, b.start + b.counter);
      b.start += b.counter;
      b.counter = 0;
    }
    
    if(!counter) m_unusedBucket = ownBucket;
    else
    {
      m_vecBucketSortInfo[ownBucket].reset(m_nbClauseInDistrib,
                                           m_nbClauseInDistrib + counter);
      m_nbClauseInDistrib += counter;
    }
    
    if(currentPos == m_sizeDistrib - 2) m_sizeDistrib -= 2; // we add nothing.
    else
    {
      m_distrib[currentPos] = l.intern();
      m_distrib[currentPos + 1] = m_sizeDistrib - currentPos - 2;
    }    
  }// createDistribWrTLit


  /**
     Collect the clause distribution. The result is stored in distrib.

     @param[in] component, the set of variables we consider.

     \return the number of elements we have in the distribution once the
     redundant clauses have been removed.
  */
  inline unsigned collectDistrib(std::vector<Var> &component)
  {
    // sort the set of clauses
    for(auto &v : component)
    {
      if(specManager.varIsAssigned(v)) continue;
      createDistribWrTLit(Lit::makeLitFalse(v));
      createDistribWrTLit(Lit::makeLitTrue(v));
    }

    // mark the clause we do not keep.
    unsigned realSizeDistrib = m_sizeDistrib;    
    for(auto &idx : m_mustUnMark)
    {
      BucketSortInfo &b = m_vecBucketSortInfo[m_markIdx[idx]];      
      m_markIdx[idx] = -1;
      m_shiftedSizeClause[b.start] = m_sizeClauses[idx];
      if(b.end != b.start + 1)
      {
        realSizeDistrib -= (b.end - b.start - 1) * specManager.getCurrentSize(idx);
        for(unsigned j = b.start + 1 ; j<b.end ; j++) m_markedAsRedundant[j] = true;
        b.end = b.start + 1;
      }
    }
    m_mustUnMark.resize(0);

    // shift the clauses indices if requiered.
    unsigned index = 0;
    for(unsigned i = 0 ; i<m_nbClauseInDistrib ; i++)
    {
      if(!m_markedAsRedundant[i])
      {
        m_distribDiffSize[m_shiftedSizeClause[i]]++;
        m_shiftedSizeClause[index] = m_shiftedSizeClause[i];
        m_shiftedIndexClause[i] = index++;
      } else m_shiftedIndexClause[i] = m_sizeDistrib;
      m_markedAsRedundant[i] = false;
    }
    m_nbClauseInDistrib = index; // resize    
    return realSizeDistrib; 
  }// collectDistrib

  
  inline void initSortBucket()
  {
    m_nbClauseInDistrib = 0;
    m_sizeDistrib = 0;
    m_unusedBucket = -1;
    m_vecBucketSortInfo.resize(0);
    for(unsigned i = 0 ; i <= m_maxSizeClause ; i++) m_distribDiffSize[i] = 0;
  } // initSortBucket

  
  inline void showListBucketSort(std::vector<BucketSortInfo> &v, std::ostream &out)
  {
    out << "size = " << v.size() << "\n";
    for(auto &e : v) out << "[" <<  e.start << " "
                         << e.end << " " 
                         << e.counter << " " 
                         << e.redirected
                         << "]";
    out << "\n";
  } // showListBucketSort

  
  /**
     Compute the number of bytes requiered to store the data.
   */
  inline unsigned computeNeededBytes(unsigned nBv, unsigned nBda,
                                     unsigned nbD, unsigned nbVar,
                                     unsigned nbEltData, unsigned nbEltDist)
  {
    return (nBv * nbVar)
        + (nBda * nbEltData)
        + (nbD * (nbEltDist<<1));   // <<1 because the size and the number.
  }

  /**
     Store the variables respecting the information of size concerning the type T
     to encode each elements and returns the pointer just after the end of the
     data.

     @param[]
   */
  template <typename U> void *storeVariables(void *data,
                                             std::vector<Var> &component)
  {
    U *p = static_cast<U *>(data);
    for(auto &v : component)
    {
      *p = static_cast<U>(v);
      p++;
    }

    return p;
  } // storeVariables


    /**
     Store the variables respecting the information of size concerning the type T
     to encode each elements and returns the pointer just after the end of the
     data.

     @param[in] data, the place where we store the information.
   */
  template <typename U> void *storeDistribInfo(void *data)
  {
    U *p = static_cast<U *>(data);
    for(unsigned i = 0 ; i <= m_maxSizeClause ; i++)
    {
      if(!m_distribDiffSize[i]) continue;
      *p = static_cast<U>(i);
      p++;
      *p = static_cast<U>(m_distribDiffSize[i]);
      p++;
    }

    return p;
  } // storeDistribInfo

  

  /**
     Store the formula representation respecting the information of size
     concerning the type T to encode each elements and returns the pointer just
     after the end of the data.

     Information about the formula is store in member variables:
      - m_sizeDistrib
      - m_distrib

     @param[in] data, the place where we store the information
     @param[in] component, is the set of variables.

     \return a pointer to the end of the data we added
  */
  template <typename U> void *storeClauses(void *data, std::vector<Var> &component)
  {
    // we map the variable to another index regarding their poistion in component.
    for(unsigned i = 0 ; i<component.size() ; i++) m_mapVar[component[i]] = i;

    // get the information about the starting offset for the different clause size.
    unsigned offSet = 0;
    unsigned memoryPlaceWrtSizeClause[m_maxSizeClause + 1];
    for(unsigned i = 0 ; i <= m_maxSizeClause ; i++)
    {
      memoryPlaceWrtSizeClause[i] = offSet;
      offSet += m_distribDiffSize[i] * i;
    }

    // allocate an offset for each clauses.
    for(unsigned i = 0 ; i<m_nbClauseInDistrib ; i++)
    {
      unsigned szClause = m_shiftedSizeClause[i];
      if(!szClause) continue;
      
      m_offsetClauses[i] = memoryPlaceWrtSizeClause[szClause];
      memoryPlaceWrtSizeClause[szClause] += szClause;      
      m_shiftedSizeClause[i] = 0;
    }

    // we store the data.
    U *p = static_cast<U *>(data);
    unsigned i = 0;
    while(i<m_sizeDistrib)
    {
      unsigned lit = m_distrib[i++];
      
      U l = static_cast<U>((m_mapVar[lit>>1]<<1) | (lit&1));
      unsigned szLitList = m_distrib[i++];

      while(szLitList)
      {        
        szLitList--;        
        
        unsigned idx = m_shiftedIndexClause[m_distrib[i++]];
        if(idx >= m_nbClauseInDistrib) continue;
        p[m_offsetClauses[idx]] = l;
        m_offsetClauses[idx]++;        
      }      
    }
    
    p += offSet;
    return p;
  } // storeClauses


  /**
     Compute from the m_distribDiffSize the number of different size and the maximum
     size.

     @param[out] maxNbSizeDistr, the clause size with the maximum number of elements.
     @parar[out] largestSizeClause, store the size of the largest clause.
     @param[out] nbDiffClauseSize, the number of different size.
     @param[out] nbLit, the number of literals in the distribution.
   */
  inline void getInfoDistributionSize(unsigned &maxNbSizeClause,
                                      unsigned &largestSizeClause,
                                      unsigned &nbDiffClauseSize,
                                      unsigned &nbLit)
  {
    largestSizeClause = 0;
    maxNbSizeClause = 0;
    nbDiffClauseSize = 0;
    for(unsigned i = 0 ; i<=m_maxSizeClause ; i++)
      if(m_distribDiffSize[i])
      {
        largestSizeClause = i;
        if(maxNbSizeClause < m_distribDiffSize[i]) maxNbSizeClause = m_distribDiffSize[i];
        nbDiffClauseSize++;
        nbLit += m_distribDiffSize[i] * i;
      }
  } // getInfoDistributionSize
  

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables
     @param[out] tmpFormula, the place where is stored the formula
     @param[out] szTmpFormula, to collect the size of the stored formula
  */
  inline void storeFormula(std::vector<Var> &component, CachedBucket<T> &b)
  {
    initSortBucket();
    collectDistrib(component);         // built the sorted formula
    
    // get information about the clause distribution
    m_lastSize = 0;
    unsigned nbLit = 0, nbVar = component.size(), maxNbSizeClause, nbDiffClauseSize, largestSizeClause;
    getInfoDistributionSize(maxNbSizeClause, largestSizeClause, nbDiffClauseSize, nbLit);

    unsigned nbOVar = this->nbOctetToEncodeInt(component.back() + 1);
    unsigned nbODistrib = this->nbOctetToEncodeInt(std::max(maxNbSizeClause, largestSizeClause));
    unsigned nbOLit = this->nbOctetToEncodeInt(nbVar << 1);

    // ask for memory
    unsigned szData = computeNeededBytes(nbOVar, nbOLit, nbODistrib,
                                         nbVar, nbLit, nbDiffClauseSize);    
    char *data = this->getArray(szData);
    void *p = data;

    // store the variables.
    switch(nbOVar)
    {
      case 1 : p = storeVariables<uint8_t>(p, component); break;
      case 2 : p = storeVariables<uint16_t>(p, component); break;
      case 4 : p = storeVariables<uint32_t>(p, component); break;
      default : throw (BucketException("Bad number of bytes",__FILE__, __LINE__));
    }
    assert(static_cast<char *>(p) == &data[nbOVar * component.size()]);
    if(!m_nbClauseInDistrib) goto fillTheBucket;
    
    // store the clause distribution of the size.
    switch(nbODistrib)
    {
      case 1 : p = storeDistribInfo<uint8_t>(p); break;
      case 2 : p = storeDistribInfo<uint16_t>(p); break;
      case 4 : p = storeDistribInfo<uint32_t>(p); break;
      default : throw (BucketException("Bad number of bytes",__FILE__, __LINE__));
    }
    assert(static_cast<char *>(p) == &data[nbOVar * component.size() + nbODistrib * (nbDiffClauseSize<<1)]);

    // store the clauses.
    switch(nbOLit)
    {
      case 1 : p = storeClauses<uint8_t>(p, component); break;
      case 2 : p = storeClauses<uint16_t>(p, component); break;
      case 4 : p = storeClauses<uint32_t>(p, component); break;
      default : throw (BucketException("Bad number of bytes",__FILE__, __LINE__));
    }
    assert(static_cast<char *>(p) == &data[szData]);

 fillTheBucket:
    // put the information into the bucket
    DataInfoCnfCl di(szData, nbVar, nbLit, nbDiffClauseSize, nbOVar, nbOLit, nbODistrib);
    assert(di.szData() == szData);
    b.set(data, di);
  }// storeFormula
};
} // d4
