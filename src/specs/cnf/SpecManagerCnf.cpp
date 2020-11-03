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

#include <iostream>
#include <algorithm>    // std::sort

#include "SpecManagerCnf.hpp"
#include "SpecManagerCnfDyn.hpp"

namespace d4
{
/**
   Constructor.
*/
SpecManagerCnf::SpecManagerCnf(int nbClause, int _nbVar, int _maxSizeClause) :
    nbVar(_nbVar), maxSizeClause(_maxSizeClause)
{
  for(unsigned i = 0 ; i <= nbVar ; i++)
  {
    inCurrentComponent.push_back(false);
    currentValue.push_back(l_Undef);
    idxComponent.push_back(0);
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
SpecManagerCnf::SpecManagerCnf(ProblemManager &p) : nbVar(p.getNbVar())
{
  initFormula(p);
  
  for(unsigned i = 0 ; i <= nbVar ; i++)
  {
    currentValue.push_back(l_Undef);
    idxComponent.push_back(0);
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
   Collect the set of literals connected to l and store the result in
   varComponent. 
   
   @param[in] l, the considered literal
   @param[in] v, the label of the previously assigned componet (0 if not assigned).
   @param[in] varComponent, the set of varaible connected to l.
   @param[in] nbComponent, the component label.
*/
void SpecManagerCnf::connectedToLit(Lit l,
                                    std::vector<int> &v,
                                    std::vector<Var> &varComponent,
                                    int nbComponent)
{
  for(auto &idx : occList[l.intern()])
  {
    if(markView[idx]) continue;
    std::vector<Lit> &c = clauses[idx];

    markView[idx] = true;
    mustUnMark.push_back(idx);

    // compute component
    for(auto &l : c )
    {
      if(currentValue[l.var()] != l_Undef || v[l.var()]) continue;
      
      varComponent.push_back(l.var());
      v[l.var()] = nbComponent;
    }
  }
}// connectedToLit


/**
   Look all the formula in order to compute the connected component
   of the formula.

   @param[out] varCo, the different connected components found
   @param[in] setOfVar, the current set of variables   
   @param[out] freeVar, the set of variables that are present in setOfVar but
   not in the problem anymore
   @param[out] notFreeVar, the difference between setOfVar and freeVar

   \return the number of component found
*/
int SpecManagerCnf::computeConnectedComponent(std::vector< std::vector<Var> > &varCo,
                                              std::vector<Var> &setOfVar,
                                              std::vector<Var> &freeVar,
                                              std::vector<Var> &notFreeVar)
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
    assert(!tmpVecVar.size());
    tmpVecVar.push_back(v);

    int cpt = 0;
    while(tmpVecVar.size())
    {
      cpt++;
      Lit l = Lit(tmpVecVar.back(), false);
      tmpVecVar.pop_back();

      connectedToLit(l, idxComponent, tmpVecVar, nbComponent);
      connectedToLit(~l, idxComponent, tmpVecVar, nbComponent);
    }

    assert(cpt > 0);
    if(cpt == 1)
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
bool SpecManagerCnf::isSatisfiedClause(unsigned idx)
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
bool SpecManagerCnf::isSatisfiedClause(std::vector<Lit> &c)
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
bool SpecManagerCnf::isNotSatisfiedClauseAndInComponent(int idx,
                                                        std::vector<bool> &inCurrentComponent)
{
  if(nbSat[idx]) return false;
  assert(watcher[idx] != lit_Undef);
  assert(!litIsAssigned(watcher[idx]));
  return inCurrentComponent[watcher[idx].var()];
}// isSatisfiedClause


void SpecManagerCnf::getCurrentClauses(std::vector<int> &idxClauses,
                                       std::vector<bool> &inComponent)
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


void SpecManagerCnf::updateCurrentFormula(std::vector<Var> &component)
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


void SpecManagerCnf::popPreviousFormula()
{
  assert(stackSize.size());
  currentSize = stackSize.back();
  stackSize.pop_back();
}// poppreviousclauseset


void SpecManagerCnf::initFormula(ProblemManager &p) 
{
  try
  {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(p);
    clauses = pcnf.getClauses();
  }
  catch (std::bad_cast& bc)
  {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }
    
  currentIdx.clear();

  for(unsigned i = 0 ; i<clauses.size() ; i++) currentIdx.push_back(i);

  currentSize = clauses.size();
  stackSize.clear();
  for(auto &val : currentValue) val = l_Undef;
}// initFormula


void SpecManagerCnf::showFormula(std::ostream &out)
{
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for(auto &cl : clauses)
  {      
    showListLit(out, cl);
    out << "0\n";
  }
}// showFormula


void SpecManagerCnf::showCurrentFormula(std::ostream &out)
{ 
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for(unsigned i = 0 ; i<clauses.size(); i++)
  {
    if(nbSat[i]) continue;    
    for(auto &l : clauses[i]) if(!litIsAssigned(l)) out << l << " ";
    out << "0\n";
  }
}// showFormula
} // d4
