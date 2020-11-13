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

#include<algorithm> 

#include "src/problem/ProblemTypes.hpp"
#include "BucketManagerCnf.hpp"

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
  unsigned *m_shiftedIndexClause;
  bool *m_markedAsRedundant;
  unsigned m_nbClauseInDistrib;
  unsigned m_sizeDistrib;
  unsigned m_capacityDistrib;
  

  // using: variables
  using BucketManagerCnf<T>::specManager;
  using BucketManagerCnf<T>::nbClauseCnf;
  using BucketManagerCnf<T>::nbVarCnf;
  using BucketManagerCnf<T>::maxSizeClause;
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
    
    m_shiftedIndexClause = new unsigned[nbClauseCnf];
    m_distrib = new unsigned[m_capacityDistrib];
    m_markedAsRedundant = new bool[nbClauseCnf];

    for(unsigned i = 0 ; i<nbClauseCnf ; i++) m_markedAsRedundant[i] = false;
  }// BucketManagerCnfCl


  /**
     Destructor.
   */
  ~BucketManagerCnfCl()
  {
    delete[] m_shiftedIndexClause;
    delete[] m_distrib;
    delete[] m_markedAsRedundant;
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
        m_mustUnMark.push_back(idx);
        m_markIdx[idx] = ownBucket;
        pushSorted(tab, nbElt++, m_nbClauseInDistrib + counter);
        counter++;        
      }else
      {
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
    initSortBucket();

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
      m_shiftedIndexClause[i] = m_markedAsRedundant[i] ? m_sizeDistrib : index++;
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
  inline unsigned computeNeededBytes(int nBv, int nBda,
                                     int nbVar, int nbEltData)
  {
    return (nBv * nbVar) + (nBda * nbEltData);
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
     Store the formula representation respecting the information of size
     concerning the type T to encode each elements and returns the pointer just
     after the end of the data.

     Information about the formula is store in member variables:
      - m_sizeDistrib
      - m_distrib

     @param[in] data, the place where we store the information
     @param[in] component, is the set of variables.
     @param[out] nbLit, used to get the number of literals of the formula.

     \return a pointer to the end of the data we added
  */
  template <typename U> void *storeClauses(void *data, std::vector<Var> &component, unsigned &nbLit)
  {
    // we map the variable to another index regarding their poistion in component.
    for(unsigned i = 0 ; i<component.size() ; i++) m_mapVar[component[i]] = i;

    U *p = static_cast<U *>(data);

    unsigned i = 0;
    while(i<m_sizeDistrib)
    {
      unsigned l = m_distrib[i++];
      p[0] = static_cast<U>((m_mapVar[l>>1]<<1) | (l&1));
      p[1] = static_cast<U>(m_distrib[i++]);

      unsigned j = i, k = 0;
      i += p[1];
      while(j<i)
      {
        if(m_shiftedIndexClause[m_distrib[j]] < m_nbClauseInDistrib)
          p[2 + k++] = static_cast<U>(m_shiftedIndexClause[m_distrib[j]]);
        j++;
      }

      p[1] = static_cast<U>(k);
      nbLit += k;
      p += (2 + k);
    }

    return p;
  } // storeClauses


  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables
     @param[out] tmpFormula, the place where is stored the formula
     @param[out] szTmpFormula, to collect the size of the stored formula
  */
  inline void storeFormula(std::vector<Var> &component, CachedBucket<T> &b)
  {
    unsigned sizeDistrib = collectDistrib(component);         // built the sorted formula

    // get information about the clause distribution
    m_lastSize = 0;
    unsigned nbLit = 0, nbVar = component.size();

    unsigned int nbOVar = this->nbOctetToEncodeInt(component.back() + 1);
    unsigned int nbOData = this->nbOctetToEncodeInt(std::max(nbVar<<1, m_nbClauseInDistrib));

    // ask for memory
    unsigned szData = computeNeededBytes(nbOVar, nbOData, nbVar, sizeDistrib);
    
    char *data = this->getArray(szData);
    void *p = data;

    // store the variables
    switch(nbOVar)
    {
      case 1 : p = storeVariables<char>(p, component); break;
      case 2 : p = storeVariables<char16_t>(p, component); break;
      default : p = storeVariables<char32_t>(p, component); break;
    }
    assert(static_cast<char *>(p) == &data[nbOVar * component.size()]);
    if(!m_nbClauseInDistrib) goto fillTheBucket;

    // store the clauses
    switch(nbOData)
    {
      case 1 : p = storeClauses<char>(p, component, nbLit); break;
      case 2 : p = storeClauses<char16_t>(p, component, nbLit); break;
      default : p = storeClauses<char32_t>(p, component, nbLit); break;
    }
    assert(static_cast<char *>(p) == &data[szData]);

 fillTheBucket:
    // put the information into the bucket
    DataInfoCnf di(szData, nbVar, nbLit, m_nbClauseInDistrib, nbOData, nbOVar);
    assert(di.szData() == szData);
    b.set(data, di);
  }// storeFormula
};
} // d4
