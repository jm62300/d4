#ifndef MODELCOUNTER_BUCKET_MANAGERHC
#define MODELCOUNTER_BUCKET_MANAGERHC

#define BIT_VECTOR 1
#define MASK_REPR 7
#define DEC_SIZE 3
#include <bitset>
#include <iostream>

using namespace std;

#include "../interfaces/BucketManagerInterface.hh"
#include "../interfaces/OccurrenceManagerInterface.hh"

#include "../utils/System.hh"
#include "../utils/SolverTypes.hh"
#include "../utils/Solver.hh"
#include "../mtl/Sort.hh"
#include "../mtl/Vec.hh"
#include "../mtl/Heap.hh"
#include "../mtl/Alg.hh"
#include "../DAG/DAG.hh"

#include "../manager/CacheBucket.hh"

#define MASK 16383
#define MASK_HEADER 1048575

template<class T> class BucketManagerHC : public BucketManagerInterface<T>
{
public:
  OccurrenceManagerInterface &occManager;
  unsigned long int nbLiterals;

  vec<bool> varInComponent;
  int nbClauseCnf, nbVarCnf;
  int strategyCache;

  /**
     Update the bucket manager to considere a new occurrence manager.

     @param[in] occM, the new occurrence manager
  */
  void updateOccManager(int nbClause, int nbVar, int maxSizeClause)
  {
    nbClauseCnf = nbClause;
    nbVarCnf = nbVar;

    varInComponent.capacity(nbVar);
    varInComponent.initialize(nbVar, false);
    this->init(nbVar, nbClause, maxSizeClause, strategyCache);
  }// updateOccManager


  /**
     Store the variables respecting the information of size concerning the type T
     to encode each elements and returns the pointer just after the end of the
     data.

     @param[]
   */
  template <typename U, typename W> void *storeData(void *data, vec<W> &value)
  {
    U *p = static_cast<U *>(data);
    for(int i = 0 ; i<value.size() ; i++)
    {
      *p = static_cast<U>(value[i]);
      p++;
    }

    return p;
  } // storeVariables

  /**
     Transfer the formula store.

     @param[in] component, the input variables
     @param[out] tmpFormula, the place where is stored the formula
     @param[out] szTmpFormula, to collect the size of the stored formula
  */
  inline void storeFormula(vec<Var> &component, CacheBucket<T> &b)
  {
    vec<int> idxClauses;

    // mark the variable in the component
    for(int i = 0 ; i<component.size() ; i++) varInComponent[component[i]] = true;

    // collect the clauses
    occManager.getCurrentClauses(idxClauses, varInComponent);
    int i, j;
    for(i = j = 0 ; i<idxClauses.size() ; i++)
    {
      if(occManager.byPass(BucketManagerInterface<T>::modeStore, idxClauses[i])) continue;
      idxClauses[j++] = idxClauses[i];
    }
    idxClauses.shrink(i - j);

    // unmark the variable in the component
    for(int i = 0 ; i<component.size() ; i++) varInComponent[component[i]] = false;

    unsigned int nbOVar = this->nbOctetToEncodeInt(component.last() + 1);
    unsigned int nbOData = idxClauses.size() ? this->nbOctetToEncodeInt((idxClauses.last()) << 1) : 0;

    // ask for memory
    unsigned szData = nbOVar * component.size() + nbOData * idxClauses.size();
    char *data = this->getArray(szData);
    void *p = data;

    // store the variables
    switch(nbOVar)
    {
      case 1 : p = storeData<char, Var>(p, component); break;
      case 2 : p = storeData<char16_t, Var>(p, component); break;
      default : p = storeData<char32_t, Var>(p, component); break;
    }
    assert(static_cast<char *>(p) == &data[nbOVar * component.size()]);
    if(!idxClauses.size()) goto fillTheBucket;

    // strore the clauses
    switch(nbOData)
    {
      case 1 : p = storeData<char, int>(p, idxClauses); break;
      case 2 : p = storeData<char16_t, int>(p, idxClauses); break;
      default : p = storeData<char32_t, int>(p, idxClauses); break;
    }

 fillTheBucket:
    // put the information into the bucket
    b.set(data, szData, component.size(), 0, idxClauses.size(), 0, nbOData, nbOVar, 0);
  }// storeFormula


public:
  /**
	Function called in order to initialized variables before using
  */
  BucketManagerHC(OccurrenceManagerInterface *occM, int strCache) :
      BucketManagerHC(occM, occM->getNbClause(), occM->getNbVariable(), occM->getMaxSizeClause(), strCache)
  {
  }// BucketManagerHC

  /**
	Function called in order to initialized variables before using
  */
  BucketManagerHC(OccurrenceManagerInterface *occM, int nbClause, int nbVar, int maxSizeClause, int strCache) :
      occManager(*occM)
  {
    assert(occM);
    strategyCache = strCache;
    updateOccManager(nbClause, nbVar, maxSizeClause);
  }// BucketManagerHC
};

#endif
