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
};

class DistribContainer
{
 private:
  unsigned *m_data;
  unsigned m_capacity;
  unsigned m_capacityLine;
  unsigned m_size;
  
 public:
  /**
     Initialize the structure.

     @param[in] capacity, the number of lines
     @param[in] capacityLine, the number of columns
   */
  void init(unsigned capacity, unsigned capacityLine)
  {
    m_capacity = capacity;
    m_capacityLine = capacityLine + 1;
    m_data = new unsigned[m_capacity * m_capacityLine]; // +1 to store the size.
    m_size = 0;
  } // constructor


  ~DistribContainer() {delete[] m_data;}
  inline void reinit(){m_size = 0;}
  inline unsigned size(){return m_size;}
  inline unsigned getInSize(unsigned idx){assert(idx < m_size); return m_data[idx * m_capacityLine];}
  
  inline void push()
  {
    assert(m_size < m_capacity);
    m_data[m_size * m_capacityLine] = 0;
    m_size++;
  } // push


  inline void pushOne(unsigned l)
  {
    assert(m_size < m_capacity);
    m_data[m_size * m_capacityLine] = 1;
    m_data[m_size * m_capacityLine + 1] = l;
    m_size++;
  }

  
  inline void pushIn(unsigned idx, unsigned l)
  {
    assert(idx < m_size);
    unsigned *tmp = &m_data[idx * m_capacityLine];
    assert(*tmp < m_capacityLine);
    
    (*tmp)++;
    tmp[*tmp] = l;
  } // pushInLast

  
  inline void resizeIn(unsigned idx, unsigned size)
  {
    assert(size < m_capacityLine && idx < m_size);
    m_data[idx * m_capacityLine] = size;
  } // resizeIn


  inline unsigned getSizeIn(unsigned idx)
  {
    assert(idx < m_size);
    return m_data[idx * m_capacityLine];
  } // getSizeIn


  inline unsigned *getArrayIn(unsigned idx)
  {
    assert(idx < m_size);
    return &m_data[(idx * m_capacityLine) + 1];
  } // getArrayIn


  inline void display(std::ostream &out)
  {
    unsigned offSet = 0;
    for(unsigned i = 0 ; i<m_size ; i++)
    {
      out << m_data[offSet] << ":";

      for(unsigned j = 0 ; j < m_data[offSet] ; j++)
        out << m_data[offSet + j + 1] << " ";
      out << "\n";
      offSet += m_capacityLine;
    }
  }
};


template<class T> class BucketManagerCnfCl : public BucketManagerCnf<T>
{
 private:
  DistribContainer m_distrib;
  
  std::vector<BucketSortInfo> m_vecBucketSortInfo;
  int m_unusedBucket;
  std::vector<unsigned long int> m_mapVar;
  unsigned long int *m_tmpKey;

  std::vector<int> m_mustUnMark;
  std::vector<int> m_markIdx;
  std::vector<bool> m_toBeConsidered;
  std::vector<unsigned> m_idInVecBucket;
  std::vector<unsigned> m_distribClauseNbVar;
  unsigned m_lastSize;

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
    m_tmpKey = new unsigned long int[nbVarCnf + 1 + (nbClauseCnf<<1)];
    m_mapVar.resize(nbVarCnf + 1, 0);
    m_markIdx.resize(nbClauseCnf, -1);
    m_toBeConsidered.resize(nbClauseCnf, false);
    m_distribClauseNbVar.resize(maxSizeClause + 1, 0);
    m_distrib.init(nbClauseCnf, maxSizeClause);
  }// BucketManagerCnfCl

  ~BucketManagerCnfCl()
  {
    delete[] m_tmpKey;
  }
  
  /**
     It is used in order to construct a sorted residual formula.

     @param[in] l, we considere the clause containing l
  */
  void createDistribWrTLit(const Lit &l)
  {
    if(!specManager.getVecIdxClause(l).size()) return;
    
    // associate a bucket to the literal.
    unsigned counter = 0;
    int ownBucket = m_unusedBucket;
    if(m_unusedBucket == -1)
    {
      ownBucket = m_vecBucketSortInfo.size();
      m_vecBucketSortInfo.push_back(BucketSortInfo(m_distrib.size()));
    } else m_unusedBucket = -1;

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
        b.counter++;        
      }
    }
    
    m_vecBucketSortInfo.resize(m_vecBucketSortInfo.size() + m_idInVecBucket.size());
    for(auto &bid : m_idInVecBucket)
    {
      BucketSortInfo &b = m_vecBucketSortInfo[bid];
      assert(b.counter);

      // we add the literal in the bucket.
      for(unsigned i = 0 ; i<b.counter ; i++) m_distrib.pushIn(b.start + i, l.intern());

      // we split out the bucket.
      m_vecBucketSortInfo[b.redirected] = BucketSortInfo(b.start, b.start + b.counter);
      b.start += b.counter;
      b.counter = 0;
    }
    
    if(!counter) m_unusedBucket = ownBucket;
    else
    {
      m_vecBucketSortInfo[ownBucket].start = m_distrib.size();
      for(unsigned i = 0 ; i<counter ; i++) m_distrib.pushOne(l.intern());
      m_vecBucketSortInfo[ownBucket].end += counter;
    }
  }// createDistribWrTLit


  /**
     Collect the clause distribution. The result is stored in distrib.

     @param[in] component, the set of variables we consider.
  */
  inline void collectDistrib(std::vector<Var> &component)
  {
    initSortBucket();

    // the set of clauses.
    // collectIdActiveClauses(component, m_idxClauses);
    // if(!m_idxClauses.size()) return;

    // for(auto &idx : m_idxClauses)
    // std::cout << specManager.getClause(idx).size() - specManager.getNbUnsat(idx) << " ";
    // std::cout << "\n";
    
    // exit(0);
    
    // sort the set of clauses
    for(auto &v : component)
    {
      if(specManager.varIsAssigned(v)) continue;
      createDistribWrTLit(Lit::makeLitFalse(v));
      createDistribWrTLit(Lit::makeLitTrue(v));
    }
    resetUnMark();

    // remove redondant clauses
    for(auto &c : m_vecBucketSortInfo)
      for(unsigned j = c.start + 1 ; j<c.end ; j++) m_distrib.resizeIn(j, 0);
  }// collectDistrib

  
  inline void initSortBucket()
  {
    m_unusedBucket = -1;
    m_vecBucketSortInfo.resize(0);
    m_distrib.reinit();
  } // initSortBucket

  
  inline void resetUnMark()
  {
    for(auto &u : m_mustUnMark) m_markIdx[u] = -1;
    m_mustUnMark.resize(0);
  }// resetUnMark



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


  inline void getInfoClDistrib(unsigned &nbLit, unsigned &nbDiffSize,
                               unsigned &nbClause, unsigned &maxDistribSz)
  {
    maxDistribSz = 0;
    for(unsigned i = 0 ; i<m_distrib.size() ; i++)
    {
      if(!m_distrib.getSizeIn(i)) continue;      
      nbLit += m_distrib.getSizeIn(i);
      assert(m_distrib.getSizeIn(i) < m_distribClauseNbVar.size());
      m_distribClauseNbVar[m_distrib.getSizeIn(i)]++;
      nbClause++;
    }

    for(unsigned int i = 0 ; i<m_distribClauseNbVar.size() ; i++)
    {
      if(m_distribClauseNbVar[i])
      {
        nbDiffSize++;
        if(m_distribClauseNbVar[i] > maxDistribSz)
          maxDistribSz = m_distribClauseNbVar[i];
        if(i > maxDistribSz) maxDistribSz = i;
        m_lastSize = i;
      }
    }
  } // getInfoClDistrib


  /**
     Compute the number of bytes requiered to store the data.
   */
  inline unsigned computeNeededBytes(int nBv, int nBda, int nBdi,
                                     int nbVar, int nbLit, int nbDiffS)
  {
    return (nBv * nbVar) + (nBda * nbLit) + (2 * nBdi * nbDiffS);
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
     Store the distribution of clause's size respecting the information of size
     concerning the type T to encode each elements and returns the pointer just
     after the end of the data.

     @param[in] data, the place where we store the information
     @param[in] distribinfo, the distribution of the clauses regarding their size
     @param[in] lastsize, the biggest claus size

     \return a pointer to the end of the data we added
   */
  template <typename U>
  void *storeDistribInfo(void *data, std::vector<unsigned> &distInfo, int last)
  {
    U *p = static_cast<U *>(data);
    for(int i = 0 ; i <= last ; i++)
    {
      if(!distInfo[i]) continue;
      *p = static_cast<U>(distInfo[i]);
      p++;
      *p = static_cast<U>(i);
      p++;

    }

    return p;
  } // storeDistribInfo


  /**
     Store the distribution of clause's size respecting the information of size
     concerning the type T to encode each elements and returns the pointer just
     after the end of the data.

     @param[in] data, the place where we store the information
     @param[in] d, the clauses
     @param[in] distribinfo, the distribution of the clauses regarding their size
     @param[in] lastsize, the biggest claus size

     \return a pointer to the end of the data we added
  */
  template <typename U> void *storeClauses(void *data,
                                           DistribContainer &d,
                                           std::vector<Var> &component,
                                           std::vector<unsigned> &distribInfo,
                                           int lastSz)
  {
    assert(nbVarCnf < m_mapVar.size());
    for(unsigned i = 0 ; i<component.size() ; i++) m_mapVar[component[i]] = i;
    
    U *p = static_cast<U *>(data);
    for(unsigned i = 0 ; i <= (unsigned) lastSz; i++)
    {
      if(!distribInfo[i]) continue;

      for(unsigned j = 0 ; j<d.size() ; j++)
      {
        if(d.getSizeIn(j) != i) continue;
        for(unsigned *tmp = d.getArrayIn(j), *end = &tmp[d.getSizeIn(j)] ; tmp != end ; tmp++)
        {
          unsigned l = *tmp; // reference a Lit (intern)
          *p = static_cast<U>((m_mapVar[l>>1] << 1) | (l&1));
          p++;
        }
      }
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
    collectDistrib(component);         // built the sorted formula

    // get information about the clause distribution
    m_lastSize = 0;
    unsigned nbLit = 0, nbDiffSize = 0, nbClause = 0, maxDistribSz = 0;
    getInfoClDistrib(nbLit, nbDiffSize, nbClause, maxDistribSz);

    unsigned int nbOVar = this->nbOctetToEncodeInt(component.back() + 1);
    unsigned int nbOData = this->nbOctetToEncodeInt((component.size() + 2) << 1);
    unsigned int nbODistrib = this->nbOctetToEncodeInt(maxDistribSz);

    // ask for memory
    unsigned szData = computeNeededBytes(nbOVar, nbOData, nbODistrib,
                                         component.size(), nbLit, nbDiffSize);
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
    if(!m_distrib.size()) goto fillTheBucket;

    // store the distribution
    switch(nbODistrib)
    {
      case 1 : p = storeDistribInfo<char>(p, m_distribClauseNbVar, m_lastSize); break;
      case 2 : p = storeDistribInfo<char16_t>(p, m_distribClauseNbVar, m_lastSize); break;
      default : p = storeDistribInfo<char32_t>(p, m_distribClauseNbVar, m_lastSize); break;
    }
    assert(static_cast<char *>(p) == &data[nbOVar * component.size() + 2 * nbODistrib * nbDiffSize]);

    // strore the clauses
    switch(nbOData)
    {
      case 1 : p = storeClauses<char>(p, m_distrib, component, m_distribClauseNbVar, m_lastSize); break;
      case 2 : p = storeClauses<char16_t>(p, m_distrib, component, m_distribClauseNbVar, m_lastSize); break;
      default : p = storeClauses<char32_t>(p, m_distrib, component, m_distribClauseNbVar, m_lastSize); break;
    }

    assert(static_cast<char *>(p) == &data[szData]);

 fillTheBucket:
    // reinit for the next run
    assert(m_lastSize < m_distribClauseNbVar.size());
    for(unsigned i = 0 ; i <= m_lastSize ; i++) m_distribClauseNbVar[i] = 0;
    
    // put the information into the bucket
    DataInfoCnf di(szData, component.size(), nbLit, nbClause, nbOData, nbOVar, nbODistrib);

    assert(di.szData() == szData);
    b.set(data, di);
  }// storeFormula


 public:

  inline void printData(std::vector<Var> &component)
  {
    printf("storeFormula:\n");
    printf("Variable: %lu\n", component.size());
    for(unsigned i = 0 ; i<component.size() ; i++) printf("%d ", component[i] + 1);
    printf("\n");
    printf("clauses: %d\n", m_distrib.size());
    for(unsigned i = 0 ; i<m_distrib.size() ; i++)
      if(m_distrib.getSizeIn(i))
      {
        
        for(unsigned *tmp = m_distrib.getArrayIn(i), *end = &tmp[m_distrib.getSizeIn(i)] ;
            tmp != end ; tmp++) 
          printf("%s%d ", ((*tmp) & 1) ? "-" : "", (*tmp)>>1);
        printf("\n");
      }
    printf("distribution info: %d\n", m_lastSize);
    for(unsigned i = 0 ; i<=m_lastSize; i++) printf("%d ", m_distribClauseNbVar[i]);
    printf("\n");
    printf("----------------------------------\n");
  }
};
} // d4
