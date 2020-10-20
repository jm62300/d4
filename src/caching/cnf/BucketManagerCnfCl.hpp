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

#ifndef d4_src_caching_cnf_BucketManagerCnfCl_hpp
#define d4_src_caching_cnf_BucketManagerCnfCl_hpp

#include "BucketManagerCnf.hpp"

namespace d4
{
typedef struct{ int start; int end;} bucketSortInfo;

template<class T> class BucketManagerCnfCl : public BucketManagerCnf<T>
{
 public:
  std::vector<bucketSortInfo> vecBucketSortIntervalle;
  std::vector<int> refCutBucket;
  std::vector<unsigned long int> mapVar;
  std::vector< std::vector<Lit> > distrib;
  std::vector<int> occInfoPos;
  unsigned long int *tmpKey;

  std::vector<bool> markView;
  std::vector<int> markIdx;
  std::vector<int> distribClauseNbVar;
  int lastSize;
  
  using BucketManagerCnf<T>::occManager;
  using BucketManagerCnf<T>::modeStore;
  using BucketManagerCnf<T>::nbClauseCnf;
  using BucketManagerCnf<T>::nbVarCnf;
  using BucketManagerCnf<T>::strategyCache;

  /**
     Update the bucket manager to considere a new occurrence manager.

     @param[in] occM, the new occurrence manager
  */
  void updateOccManager(int nbClause, int nbVar, int maxSizeClause)
  {
    nbClauseCnf = nbClause;
    nbVarCnf = nbVar;

    occInfoPos.resize(nbVarCnf, -1);    
    tmpKey = (unsigned long int*) realloc(tmpKey, (nbVarCnf + (nbClauseCnf<<1)) *
                                          sizeof(unsigned long int));
    mapVar.resize(nbVarCnf, 0);
    markView.resize(nbClauseCnf, false);
    markIdx.resize(nbClauseCnf, -1);
    distribClauseNbVar.resize(maxSizeClause, 0);
    
    this->init(nbVar, nbClause, maxSizeClause, strategyCache);
  }// updateOccManager


  /**
     It is used in order to construct a sorted residual formula.

     @param[in] l, we considere the clause containing l
  */
  void createDistribWrTLit(Lit l)
  {
    assert(occManager);
    std::vector<int> &idxClauses = occManager->getVecIdxClause(l);
    int ownBucket = -1;

    for(int j = 0 ; j<idxClauses.size() ; j++)
    {
      int idx = idxClauses[j];

      if(modeStore == NT && !occManager->getNbUnsat(idx)) continue;
      std::vector<Lit> &c = occManager->getClause(idx);
      if(modeStore == NB && c.size() <= 2) continue;

      assert(idx < markIdx.size());
      if(!markView[idx])
      {
        markView[idx] = true;
        mustUnMark.push_back(idx);

        if(ownBucket == -1) // create its own bucket
        {
          ownBucket = vecBucketSortIntervalle.size();
          vecBucketSortIntervalle.push_back((bucketSortInfo) {distrib.size(), distrib.size()});
          refCutBucket.push_back(ownBucket);
        }

        markIdx[idx] = ownBucket;
        distrib.push_back();
        (distrib.back()).resize(0);
        (distrib.back()).push_back(l);
        vecBucketSortIntervalle[ownBucket].end++;
      }else
      {
        int bkNew = markIdx[idx], bkOld = bkNew;
        assert(bkNew < refCutBucket.size());

        if(refCutBucket[bkNew] == bkNew)
        {
          bucketSortInfo tmp = vecBucketSortIntervalle[bkNew];
          tmp.end = tmp.start;
          vecBucketSortIntervalle.push_back(tmp);
          bkNew = vecBucketSortIntervalle.size() - 1;
          refCutBucket.push_back(bkNew);
          refCutBucket[bkOld] = bkNew;
        }else bkNew = refCutBucket[bkNew];

        markIdx[idx] = bkNew;
        vecBucketSortIntervalle[bkOld].start++;
        bucketSortInfo &currB = vecBucketSortIntervalle[bkNew];

        assert(distrib[currB.end].size() < distrib[currB.end].capacity());
        distrib[currB.end++].push_back(l);
      }
    }

    for(int j = 0 ; j<refCutBucket.size() ; j++) refCutBucket[j] = j;
  }// createDistribWrTLit


  inline void initSortBucket()
  {
    vecBucketSortIntervalle.clear();
    refCutBucket.clear();
    distrib.clear();
  }


  inline void showListBucketSort(std::vector<bucketSortInfo> &v)
  {
    for(unsigned i = 0 ; i<v.size() ; i++) printf("[%d %d]", v[i].start, v[i].end);
    printf("\n");
  }

  std::vector<int> mustUnMark;
  inline void resetUnMark()
  {
    for(int i = 0 ; i<mustUnMark.size() ; i++) markView[mustUnMark[i]] = false;
    mustUnMark.resize(0);
  }// resetUnMark

  /**
     Collect the clause distribution
  */
  inline void collectDistrib(std::vector<Var> &component)
  {
    assert(occManager);
    initSortBucket();

    // sort the set of clauses
    for(unsigned i = 0 ; i<component.size() ; i++)
    {
      if(occManager->varIsAssigned(component[i])) continue;
      Lit l = Lit(component[i], false);
      createDistribWrTLit(l);
      createDistribWrTLit(~l);
    }
    resetUnMark();


    // remove redondant clauses
    for(int i = 0 ; i<vecBucketSortIntervalle.size() ; i++)
    {
      bucketSortInfo &c = vecBucketSortIntervalle[i];
      for(int j = c.start + 1 ; j<c.end ; j++) distrib[j].resize(0);
    }
  }// collectDistrib


  inline void getInfoClDistrib(unsigned &nbLit, unsigned &nbDiffSize, unsigned &nbClause,
                               unsigned &maxDistribSz)
  {
    for(int i = 0 ; i<distribClauseNbVar.size() ; i++) assert(distribClauseNbVar[i] == 0);

    maxDistribSz = 0;
    for(int i = 0 ; i<distrib.size() ; i++)
    {
      if(!distrib[i].size()) continue;
      nbLit += distrib[i].size();
      distribClauseNbVar[distrib[i].size()]++;
      nbClause++;
    }

    for(unsigned int i = 0 ; i<distribClauseNbVar.size() ; i++)
    {
      if(distribClauseNbVar[i])
      {
        nbDiffSize++;
        if(distribClauseNbVar[i] > maxDistribSz) maxDistribSz = distribClauseNbVar[i];
        if(i > maxDistribSz) maxDistribSz = i;
        lastSize = i;
      }
    }
  } // getInfoClDistrib


  /**
     Compute the number of bytes requiered to store the data.
   */
  inline unsigned computeNeededBytes(int nBv, int nBda, int nBdi, int nbVar, int nbLit, int nbDiffS)
  {
    return (nBv * nbVar) + (nBda * nbLit) + (2 * nBdi * nbDiffS);
  }

  /**
     Store the variables respecting the information of size concerning the type T
     to encode each elements and returns the pointer just after the end of the
     data.

     @param[]
   */
  template <typename U> void *storeVariables(void *data, std::vector<Var> &component)
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
  template <typename U> void *storeDistribInfo(void *data, std::vector<int> &distribInfo, int last)
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
  template <typename U> void *storeClauses(void *data, std::vector< std::vector<Lit> > &d,
                                           std::vector<Var> &component,
                                           std::vector<int> &distribInfo, int lastSz)
  {
    assert(nbVarCnf <= mapVar.size());
    for(unsigned i = 0 ; i<component.size() ; i++) mapVar[component[i]] = i;

    int cpt = 0;

    U *p = static_cast<U *>(data);
    for(unsigned i = 0 ; i <= (unsigned) lastSz; i++)
    {
      if(!distribInfo[i]) continue;

      for(unsigned j = 0 ; j<d.size() ; j++)
      {
        if(d[j].size() != i) continue;

        for(unsigned k = 0 ; k < d[j].size() ; k++)
        {
          Lit l = d[j][k];
          *p = static_cast<U>((mapVar[l.var()] << 1) | l.sign());
          p++;

          cpt++;
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
    lastSize = 0;
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
    if(!distrib.size()) goto fillTheBucket;

    // store the distribution
    switch(nbODistrib)
    {
      case 1 : p = storeDistribInfo<char>(p, distribClauseNbVar, lastSize); break;
      case 2 : p = storeDistribInfo<char16_t>(p, distribClauseNbVar, lastSize); break;
      default : p = storeDistribInfo<char32_t>(p, distribClauseNbVar, lastSize); break;
    }
    assert(static_cast<char *>(p) == &data[nbOVar * component.size() + 2 * nbODistrib * nbDiffSize]);

    // strore the clauses
    switch(nbOData)
    {
      case 1 : p = storeClauses<char>(p, distrib, component, distribClauseNbVar, lastSize); break;
      case 2 : p = storeClauses<char16_t>(p, distrib, component, distribClauseNbVar, lastSize); break;
      default : p = storeClauses<char32_t>(p, distrib, component, distribClauseNbVar, lastSize); break;
    }

    assert(static_cast<char *>(p) == &data[szData]);

 fillTheBucket:
    // reinit for the next run
    assert(lastSize < distribClauseNbVar.size());
    for(int i = 0 ; i <= lastSize ; i++) distribClauseNbVar[i] = 0;

    // put the information into the bucket
    b.set(data, szData, component.size(), nbLit, nbClause, nbDiffSize<<1, nbOData, nbOVar, nbODistrib);
  }// storeFormula


 public:
  /**
     Function called in order to initialized variables before using
  */
  BucketManagerCnfCl(CnfOccurrenceManager *occM, int mdStore, int strCache) :
      BucketManagerCnf<T>::BucketManagerCnf(occM, mdStore, strCache)
  {
    tmpKey = NULL;
  }// BucketManager

  ~BucketManagerCnfCl()
  {
    if(tmpKey) free(tmpKey);
    distrib.resize(nbClauseCnf);
    for(int i = 0 ; i<distrib.size() ; i++) distrib[i].clear();
    distrib.clear();
  }


  inline void printData(std::vector<Var> &component)
  {
    printf("storeFormula:\n");
    printf("Variable: %d\n", component.size());
    for(unsigned i = 0 ; i<component.size() ; i++) printf("%d ", component[i] + 1);
    printf("\n");
    printf("clauses: %d\n", distrib.size());
    for(int i = 0 ; i<distrib.size() ; i++)
      if(distrib[i].size())
      {
        for(int j = 0 ; j<distrib[i].size() ; j++)
          printf("%d ", distrib[i][j].human());
        printf("\n");
      }
    printf("distribution info: %d\n", lastSize);
    for(int i = 0 ; i<=lastSize; i++) printf("%d ", distribClauseNbVar[i]);
    printf("\n");
    printf("----------------------------------\n");
  }
};


}

#endif
