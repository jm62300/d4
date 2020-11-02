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
typedef struct{ unsigned start; unsigned end;} bucketSortInfo;

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
    m_capacityLine = capacityLine;
    m_data = new unsigned[m_capacity * (m_capacityLine + 1)]; // +1 to store the size.
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
    return &m_data[idx * m_capacityLine + 1];
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
  
  std::vector<bucketSortInfo> m_vecBucketSortIntervalle;
  std::vector<unsigned> m_refCutBucket;
  std::vector<unsigned long int> m_mapVar;
  unsigned long int *m_tmpKey;

  std::vector<bool> m_markView;
  std::vector<int> m_markIdx;
  std::vector<unsigned> m_distribClauseNbVar;
  unsigned m_lastSize;
  
  using BucketManagerCnf<T>::specManager;
  using BucketManagerCnf<T>::modeStore;
  using BucketManagerCnf<T>::nbClauseCnf;
  using BucketManagerCnf<T>::nbVarCnf;
  using BucketManagerCnf<T>::maxSizeClause;

 public:
  /**
     Function called in order to initialized variables before using
  */
  BucketManagerCnfCl(SpecManagerCnf &occM, int mdStore, unsigned sizePage) :
      BucketManagerCnf<T>::BucketManagerCnf(occM, mdStore, sizePage)
  {
    m_tmpKey = new unsigned long int[nbVarCnf + 1 + (nbClauseCnf<<1)];
    m_mapVar.resize(nbVarCnf + 1, 0);
    m_markView.resize(nbClauseCnf, false);
    m_markIdx.resize(nbClauseCnf, -1);
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
  void createDistribWrTLit(Lit l)
  {
    std::vector<int> &idxClauses = specManager.getVecIdxClause(l);
    int ownBucket = -1;

    for(unsigned j = 0 ; j<idxClauses.size() ; j++)
    {
      unsigned idx = idxClauses[j];

      if(modeStore == NT && !specManager.getNbUnsat(idx)) continue;
      std::vector<Lit> &c = specManager.getClause(idx);
      if(modeStore == NB && c.size() <= 2) continue;

      assert(idx < m_markIdx.size());
      if(!m_markView[idx])
      {
        m_markView[idx] = true;
        mustUnMark.push_back(idx);

        if(ownBucket == -1) // create its own bucket
        {
          ownBucket = m_vecBucketSortIntervalle.size();
          m_vecBucketSortIntervalle.push_back((bucketSortInfo) {m_distrib.size(), m_distrib.size()});
          m_refCutBucket.push_back(ownBucket);
        }

        m_markIdx[idx] = ownBucket;
        m_distrib.push();
        m_distrib.pushIn(m_distrib.size() - 1, l.intern());
        m_vecBucketSortIntervalle[ownBucket].end++;
      }else
      {
        unsigned bkNew = m_markIdx[idx], bkOld = bkNew;
        assert(bkNew < m_refCutBucket.size());

        if(m_refCutBucket[bkNew] == bkNew)
        {
          bucketSortInfo tmp = m_vecBucketSortIntervalle[bkNew];
          tmp.end = tmp.start;
          m_vecBucketSortIntervalle.push_back(tmp);
          bkNew = m_vecBucketSortIntervalle.size() - 1;
          m_refCutBucket.push_back(bkNew);
          m_refCutBucket[bkOld] = bkNew;
        }else bkNew = m_refCutBucket[bkNew];

        m_markIdx[idx] = bkNew;
        m_vecBucketSortIntervalle[bkOld].start++;
        bucketSortInfo &currB = m_vecBucketSortIntervalle[bkNew];
        
        m_distrib.pushIn(currB.end++, l.intern());
      }
    }

    for(unsigned j = 0 ; j<m_refCutBucket.size() ; j++) m_refCutBucket[j] = j;
  }// createDistribWrTLit


  inline void initSortBucket()
  {
    m_vecBucketSortIntervalle.clear();
    m_refCutBucket.clear();
    m_distrib.reinit();
  } // initSortBucket


  inline void showListBucketSort(std::vector<bucketSortInfo> &v)
  {
    for(unsigned i = 0 ; i<v.size() ; i++) printf("[%d %d]", v[i].start, v[i].end);
    printf("\n");
  }

  std::vector<int> mustUnMark;
  inline void resetUnMark()
  {
    for(auto &u : mustUnMark) m_markView[u] = false;
    mustUnMark.clear();
  }// resetUnMark

  /**
     Collect the clause distribution
  */
  inline void collectDistrib(std::vector<Var> &component)
  {
    initSortBucket();

    // sort the set of clauses
    for(unsigned i = 0 ; i<component.size() ; i++)
    {
      if(specManager.varIsAssigned(component[i])) continue;      
      Lit l = Lit(component[i], false);
      
      createDistribWrTLit(l);
      createDistribWrTLit(~l);
    }
    resetUnMark();


    // remove redondant clauses
    for(unsigned i = 0 ; i<m_vecBucketSortIntervalle.size() ; i++)
    {
      bucketSortInfo &c = m_vecBucketSortIntervalle[i];
      for(unsigned j = c.start + 1 ; j<c.end ; j++) m_distrib.resizeIn(j, 0);
    }
  }// collectDistrib


  inline void getInfoClDistrib(unsigned &nbLit, unsigned &nbDiffSize,
                               unsigned &nbClause,
                               unsigned &maxDistribSz)
  {
    maxDistribSz = 0;
    for(unsigned i = 0 ; i<m_distrib.size() ; i++)
    {
      if(!m_distrib.getSizeIn(i)) continue;      
      nbLit += m_distrib.getSizeIn(i);
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
    for(unsigned i = 0 ; i<component.size() ; i++)
    {
      *p = static_cast<U>(component[i]);
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
  template <typename U> void *storeDistribInfo(void *data,
                                               std::vector<unsigned> &distribInfo,
                                               int last)
  {
    U *p = static_cast<U *>(data);
    for(int i = 0 ; i <= last ; i++)
    {
      if(!distribInfo[i]) continue;
      *p = static_cast<U>(distribInfo[i]);
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
    DataInfoCnf di(szData, component.size(), nbLit, nbClause,
                   nbDiffSize<<1, nbOData, nbOVar, nbODistrib);
    b.set(data, di);
  }// storeFormula


 public:

  inline void printData(std::vector<Var> &component)
  {
    printf("storeFormula:\n");
    printf("Variable: %d\n", component.size());
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
    for(int i = 0 ; i<=m_lastSize; i++) printf("%d ", m_distribClauseNbVar[i]);
    printf("\n");
    printf("----------------------------------\n");
  }
};
} // d4
