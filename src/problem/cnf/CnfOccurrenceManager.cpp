#include <iostream>
#include <algorithm>    // std::sort

#include "CnfOccurrenceManager.hpp"
#include "CnfOccurrenceManager.hpp"

namespace d4
{
/**
   Constructor.
 */
CnfOccurrenceManager::CnfOccurrenceManager(int nbClause, int _nbVar, int _maxSizeClause) :
    nbVar(_nbVar), maxSizeClause(_maxSizeClause)
{
  for(unsigned i = 0 ; i<nbVar ; i++)
    {
      inCurrentComponent.push_back(false);
      currentValue.push_back(l_Undef);
      idxComponent.push_back(0);
      tmpVecVar.push_back(0);
      occList.push_back(std::vector<int>());
      occList.push_back(std::vector<int>());
      tmpMark.push_back(false);
    }

  mustUnMark.reserve(nbClause);
  for(int i = 0 ; i<nbClause ; i++)
    {      
      markView.push_back(false);
      nbUnsat.push_back(0);
      nbSat.push_back(0);
      watcher.push_back(lit_Undef);
    }
}// construtor


/**
   Constructor.
 */
CnfOccurrenceManager::CnfOccurrenceManager(ProblemManager &p) : nbVar(p.getNbVar())
{
  initFormula(p);

  for(unsigned i = 0 ; i<nbVar ; i++)
    {
      currentValue.push_back(l_Undef);
      idxComponent.push_back(0);
      tmpVecVar.push_back(0);
      occList.push_back(std::vector<int>());
      occList.push_back(std::vector<int>());
      tmpMark.push_back(false);
      inCurrentComponent.push_back(false);
    }

  mustUnMark.reserve(clauses.size());

  if(!clauses.size()) return;
  maxSizeClause = clauses[0].size();
  for(auto &cl : clauses)
    {
      if(cl.size() > maxSizeClause) maxSizeClause = cl.size();
      markView.push_back(false);
      nbUnsat.push_back(0);
      nbSat.push_back(0);
      watcher.push_back(cl[0]);
    }
}// construtor


/**
   @param[in] l, the considered literal
   WARNING: varConnected references tmpVecVar. Thus, it is already allocate.
*/
int CnfOccurrenceManager::connectedToLit(Lit l, std::vector<int> &v,
                                         std::vector<Var> &varComponent, int nbComponent)
{
  int cpt = 0;
  for(auto &idx : occList[l.intern()])
    {
      std::vector<Lit> &c = clauses[idx];
      if(markView[idx]) continue;
         
      cpt++;
      markView[idx] = true;
      mustUnMark.push_back(idx);

      // compute component
      for(auto &l : c )
        {
          if(currentValue[l.var()] != l_Undef) continue;

          Var vTmp = l.var();
          if(!v[vTmp])
            {
              varComponent.push_back(vTmp);
              v[vTmp] = nbComponent;
            }
        }
    }
  return cpt;
}// connectedToLit


/**
   Look all the formula in order to compute the connected component
   of the formula.

   @param[out] varCo, the different connected components found
   @param[in] setOfVar, the current set of variables
   @param[out] freeVar, the set of variables that are present in setOfVar but not in the problem anymore
   @param[out] notFreeVar, the difference between setOfVar and freeVar

   \return the number of component found
 */
int CnfOccurrenceManager::computeConnectedComponent(std::vector< std::vector<Var> > &varCo,
                                                    std::vector<Var> &setOfVar,
                                                    std::vector<Var> &freeVar, std::vector<Var> &notFreeVar)
{
  freeVar.clear();
  int nbComponent = 0;

  for(auto &v : setOfVar)
    {
      if(currentValue[v] != l_Undef || idxComponent[v]) continue;

      // index a new composant
      nbComponent++;
      idxComponent[v] = nbComponent;

      // save the variables of connected component
      tmpVecVar.resize(0);
      tmpVecVar.push_back(v);

      int nbClausesInComponent = 0;
      for(auto &vv : tmpVecVar)
        {
          Lit l = Lit(vv, false);
          nbClausesInComponent += connectedToLit(l, idxComponent, tmpVecVar, nbComponent);
          nbClausesInComponent += connectedToLit(~l, idxComponent, tmpVecVar, nbComponent);
        }

      if(tmpVecVar.size() <= 1)
        {
          idxComponent[v] = 0;
          nbComponent--; // it is alone ...
        }
    }

  resetUnMark();

  for(int i = 0 ; i<nbComponent ; i++) varCo.push_back(std::vector<Var>());
  for(auto &v : setOfVar)
    {
      if(idxComponent[v])
        {
          assert(nbComponent);
          varCo[idxComponent[v] - 1].push_back(v);
          notFreeVar.push_back(v);
        }
      if(!idxComponent[v] && currentValue[v] == l_Undef) freeVar.push_back(v);
      idxComponent[v] = 0;
    }

  return nbComponent;
}// computeConnectedComponent


/**
   Test if a given clause is actually satisfied under the current
   interpretation.

   @param[in] idx, the clause index.

   \return true if the clause is satisfied, false otherwise.
 */
inline bool CnfOccurrenceManager::isSatisfiedClause(unsigned idx)
{
  assert(idx < clauses.size());
  return nbSat[idx];
}// isSatisfiedClause


/**
   Test if a given clause is actually satisfied under the current
   interpretation.

   @param[in] idx, the clause index.

   \return true if the clause is satisfied, false otherwise.
 */
inline bool CnfOccurrenceManager::isSatisfiedClause(std::vector<Lit> &c)
{
  for(auto &l : c)
    {
      if(!litIsAssigned(l)) continue;
      if(l.sign() && currentValue[l.var()] == l_False) return true;
      if(!l.sign() && currentValue[l.var()] == l_True) return true;
    }

  return false;
}// isSatisfiedClause


/**
   Test at the same time if a given clause is actually satisfied under
   the current interpretation and if its set of variables belong to the
   current component that is represented as a boolean map given in
   parameter.

   @param[in] idx, the clause index.
   @param[in] currentComponent, currentComponent[var] is true when var is in the
   current component, false otherwise.

   \return true if the clause is satisfied, false otherwise.
 */
inline bool CnfOccurrenceManager::isNotSatisfiedClauseAndInComponent(int idx,
                                                                     std::vector<bool> &inCurrentComponent)
{
  if(nbSat[idx]) return false;
  assert(watcher[idx] != lit_Undef);
  assert(!litIsAssigned(watcher[idx]));
  return inCurrentComponent[watcher[idx].var()];
}// isSatisfiedClause


void CnfOccurrenceManager::getCurrentClauses(std::vector<int> &idxClauses, std::vector<bool> &inComponent)
{
  idxClauses.resize(0);
  for(int i = 0 ; i<currentSize ; i++)
  {
    idxClauses.push_back(currentIdx[i]);
    assert(!nbSat[currentIdx[i]]);
    assert(isNotSatisfiedClauseAndInComponent(currentIdx[i], inComponent));
  }
  
  std::sort(idxClauses.begin(), idxClauses.end());
}// getCurrentclauses


void CnfOccurrenceManager::updateCurrentClauseSet(std::vector<Var> &component)
{
  for(auto &v : component) inCurrentComponent[v] = true;

  stackSize.push_back(currentSize);
  int i = 0;
  while(i < currentSize)
  {
    if(isNotSatisfiedClauseAndInComponent(currentIdx[i], inCurrentComponent)) i++;
    else
    {
      currentSize--;
      int tmp = currentIdx[currentSize];
      currentIdx[currentSize] = currentIdx[i];
      currentIdx[i] = tmp;
    }
  }
  
  for(auto &v : component) inCurrentComponent[v] = false;
}// updatecurrentclauseset


void CnfOccurrenceManager::popPreviousClauseSet()
{
  assert(stackSize.size());
  currentSize = stackSize.back();
  stackSize.pop_back();
}// poppreviousclauseset

}
