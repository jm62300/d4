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
SpecManagerCnf::SpecManagerCnf(int nbClause, int _nbVar, int _m_maxSizeClause) :
    m_nbVar(_nbVar), m_maxSizeClause(_m_maxSizeClause)
{
  // variables:
  m_inCurrentComponent.resize(m_nbVar + 1, false);
  m_currentValue.resize(m_nbVar + 1, l_Undef);
  m_idxComponent.resize(m_nbVar + 1, 0);

  // occurrences:
  m_occListBin.resize((m_nbVar + 1) << 1, std::vector<int>());
  m_occListNotBin.resize((m_nbVar + 1) << 1, std::vector<int>());

  // clauses:
  m_mustUnMark.reserve(nbClause);
  m_markView.resize(nbClause, false);
  m_infoClauses.resize(nbClause);
}// construtor


/**
   Constructor.
*/
SpecManagerCnf::SpecManagerCnf(ProblemManager &p) : m_nbVar(p.getNbVar())
{
  initFormula(p);

  // variables:
  m_inCurrentComponent.resize(m_nbVar + 1, false);
  m_currentValue.resize(m_nbVar + 1, l_Undef);
  m_idxComponent.resize(m_nbVar + 1, 0);

  // occurrences:
  m_occListBin.resize((m_nbVar + 1) << 1, std::vector<int>());
  m_occListNotBin.resize((m_nbVar + 1) << 1, std::vector<int>());
  
  // clauses:
  unsigned nbClause = m_clauses.size();
  m_mustUnMark.reserve(nbClause);
  m_markView.resize(nbClause, false);

  m_infoClauses.resize(nbClause);
  m_maxSizeClause = 0;
  if(!m_clauses.size()) return;

  // get the size of the largest clause.
  m_maxSizeClause = m_clauses[0].size();  
  for(unsigned i = 0 ; i<m_clauses.size() ; i++)
  {
    std::vector<Lit> &cl = m_clauses[i];
    std::vector<std::vector<int> > &listOcc = (cl.size() == 2) ?
                                              m_occListBin :
                                              m_occListNotBin;
    for(auto &l : cl) listOcc[l.intern()].push_back(i);
    if(cl.size() > m_maxSizeClause) m_maxSizeClause = cl.size();
    m_infoClauses[i].watcher = cl[0];
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
  for(unsigned i = 0 ; i<2 ; i++)
  {
    std::vector<int> &listIndex = (i) ? getVecIdxClauseBin(l) :
                                  getVecIdxClauseNotBin(l);
      
    for(auto &idx : listIndex)
    {
      if(m_markView[idx]) continue;
      m_markView[idx] = true;
      m_mustUnMark.push_back(idx);
    
      // compute component
      for(auto &l : m_clauses[idx])
      {
        if(m_currentValue[l.var()] != l_Undef || v[l.var()]) continue;
      
        varComponent.push_back(l.var());
        v[l.var()] = nbComponent;
      }
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
  freeVar.resize(0);
  
  int nbComponent = 0;
  for(auto &v : setOfVar)
  {
    if(m_currentValue[v] != l_Undef || m_idxComponent[v]) continue;

    // index a new composant
    nbComponent++;
    m_idxComponent[v] = nbComponent;

    // save the variables of connected component
    assert(!m_tmpVecVar.size());
    m_tmpVecVar.push_back(v);

    int cpt = 0;
    while(m_tmpVecVar.size())
    {
      cpt++;
      Lit l = Lit::makeLit(m_tmpVecVar.back(), false);
      m_tmpVecVar.pop_back();

      if(getNbOccurrence(l)) connectedToLit(l, m_idxComponent, m_tmpVecVar, nbComponent);
      if(getNbOccurrence(~l)) connectedToLit(~l, m_idxComponent, m_tmpVecVar, nbComponent);
    }

    assert(cpt > 0);
    if(cpt == 1)
    {
      m_idxComponent[v] = 0;
      nbComponent--; // it is alone ...
    }
  }

  resetUnMark();
  
  varCo.resize(nbComponent);
  for(auto &v : setOfVar)
  {
    if(m_idxComponent[v])
    {
      varCo[m_idxComponent[v] - 1].push_back(v);
      notFreeVar.push_back(v);
      assert(nbComponent);
    } else if(m_currentValue[v] == l_Undef) freeVar.push_back(v);
    
    m_idxComponent[v] = 0;
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
  assert(idx < m_clauses.size());
  return m_infoClauses[idx].nbSat;
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
    if(l.sign() && m_currentValue[l.var()] == l_False) return true;
    if(!l.sign() && m_currentValue[l.var()] == l_True) return true;
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
                                                        std::vector<bool> &m_inCurrentComponent)
{
  if(m_infoClauses[idx].nbSat) return false;
  assert(m_infoClauses[idx].watcher != lit_Undef);
  assert(!litIsAssigned(m_infoClauses[idx].watcher));
  return m_inCurrentComponent[m_infoClauses[idx].watcher.var()];
}// isSatisfiedClause


void SpecManagerCnf::getCurrentClauses(std::vector<unsigned> &idxClauses,
                                       std::vector<Var> &component)
{
  idxClauses.resize(0);
  for(auto &v : component) m_inCurrentComponent[v] = true;
  for(unsigned i = 0 ; i<m_clauses.size() ; i++)
  {
    if(isNotSatisfiedClauseAndInComponent(i, m_inCurrentComponent))
      idxClauses.push_back(i);
  }
  for(auto &v : component) m_inCurrentComponent[v] = false;
}// getCurrentclauses


void SpecManagerCnf::getCurrentClausesNotBin(
    std::vector<unsigned> &idxClauses,
    std::vector<Var> &component)
{
  idxClauses.resize(0);
  for(auto &v : component) m_inCurrentComponent[v] = true;
  for(auto &i : m_clausesNotBin)
  {    
    if(isNotSatisfiedClauseAndInComponent(i, m_inCurrentComponent))
      idxClauses.push_back(i);
  }
  for(auto &v : component) m_inCurrentComponent[v] = false;
}// getCurrentclauses



void SpecManagerCnf::initFormula(ProblemManager &p) 
{
  try
  {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(p);
    m_clauses = pcnf.getClauses();

    for(unsigned i = 0 ; i<m_clauses.size() ; i++)
      if(m_clauses[i].size() > 2) m_clausesNotBin.push_back(i);
  }
  catch (std::bad_cast& bc)
  {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }
    
  for(auto &val : m_currentValue) val = l_Undef;
}// initFormula


void SpecManagerCnf::showFormula(std::ostream &out)
{
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for(auto &cl : m_clauses)
  {      
    showListLit(out, cl);
    out << "0\n";
  }
}// showFormula


void SpecManagerCnf::showTrail(std::ostream &out)
{
  for(int i = 0 ; i<getNbVariable() ; i++)
  {
    if(!varIsAssigned(i)) continue;
    Lit l = Lit::makeLit(i, false);
    if(litIsAssignedToTrue(l)) out << l << " "; else out << ~l << " ";
  }
  out << "\n";
}// showFormula


void SpecManagerCnf::showCurrentFormula(std::ostream &out)
{ 
  out << "p cnf " << getNbVariable() << " " << getNbClause() << "\n";
  for(unsigned i = 0 ; i<m_clauses.size(); i++)
  {
    if(m_infoClauses[i].nbSat) continue;    
    for(auto &l : m_clauses[i]) if(!litIsAssigned(l)) out << l << " ";
    out << "0\n";
  }
}// showFormula
} // d4
