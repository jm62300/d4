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

#include "SpecManagerCnf.hpp"
#include "SpecManagerCnfDyn.hpp"

namespace d4
{
/**
   OccurrenceManager constructor. This function initialized the
   structures used.

   @param[in] nbC, the maximum number of clauses allowed
   @param[in] nbV, the number of variables in the problem
   @param[in] maxClSz, the largest clause size
 */
SpecManagerCnfDyn::SpecManagerCnfDyn(int nbC, int nbV, int maxClSz) :
    SpecManagerCnf(nbC, nbV, maxClSz)
{
}// SpecManagerCnfDyn

/**
   OccurrenceManager constructor. This function initialized the
   structures used.

   @param[in] _clauses, the set of clauses
   @param[in] _nbVar, the number of variables in the problem
 */
SpecManagerCnfDyn::SpecManagerCnfDyn(ProblemManager &p) : SpecManagerCnf(p)
{
  // create the occurrence list
  for(unsigned i = 0 ; i<clauses.size() ; i++)
    for(auto &l : clauses[i]) occList[l.intern()].push_back(i);
}// SpecManagerCnfDyn


/**
   Initiliaze the occurrence manager with a new set of clauses.
 */
void SpecManagerCnfDyn::initFormula(ProblemManager &p)
{
  SpecManagerCnf::initFormula(p);
  
  // clear the occurrence list
  for(auto &list : occList) list.clear();

  // construct the occurrence list
  for(unsigned i = 0 ; i<clauses.size() ; i++)
  {
    for(auto &l : clauses[i]) occList[l.intern()].push_back(i);
    if(clauses[i].size() > maxSizeClause) maxSizeClause = clauses[i].size();
  }
  
  mustUnMark.reserve(clauses.size());
  for(unsigned i = 0 ; i<clauses.size() ; i++)
  {
    if(markView.size() > i) markView[i] = false; else markView.push_back(false);
    if(nbUnsat.size() > i) nbUnsat[i] = 0; else nbUnsat.push_back(0);
    if(nbSat.size() > i) nbSat[i] = 0; else nbSat.push_back(0);
    if(watcher.size() > i) watcher[i] = clauses[i][0]; else watcher.push_back(clauses[i][0]);
  }
}// initFormula


/**
   Update the occurrence list w.r.t. a new set of assigned variables.
   It's important that the order is conserved between the moment where
   we assign and the moment we unassign.

   @param[in] lits, the new assigned variables
 */
void SpecManagerCnfDyn::preUpdate(std::vector<Lit> &lits)
{
  std::vector<int> reviewWatcher;

  for(auto &l : lits)
    {
      currentValue[l.var()] = l.sign() ? l_False : l_True;
      
      for(auto &idxCl : occList[l.intern()])
        {
          nbSat[idxCl]++;
          for(auto &ll : clauses[idxCl])
            if(currentValue[ll.var()] == l_Undef)
              removeIdxFromOccList(occList[ll.intern()], idxCl);            
        }
      
      for(auto &idxCl : occList[(~l).intern()])
      {
        nbUnsat[idxCl]++;
        if(watcher[idxCl] == ~l) reviewWatcher.push_back(idxCl);
      }
    }

  // we search another non assigned literal if requiered
  for(auto &idxCl : reviewWatcher)
  {
    if(nbSat[idxCl]) continue;

    for(auto &l : clauses[idxCl])
    {
      if(currentValue[l.var()] == l_Undef)
      {
        watcher[idxCl] = l;
        break;
      }
    }    
  }

  stackSize.push_back(currentSize);
  int i = 0;
  while(i<currentSize)
  {
    if(!nbSat[currentIdx[i]]) i++;
    else
    {
      currentSize--;
      int tmp = currentIdx[currentSize];
      currentIdx[currentSize] = currentIdx[i];
      currentIdx[i] = tmp;
    }
  }
}// preUpdate


/**
   Update the occurrence list w.r.t. a new set of unassigned variables.
   It's important that the order is conserved between the moment where
   we assign and the moment we unassign.

   @param[in] lits, the new assigned variables
 */
void SpecManagerCnfDyn::postUpdate(std::vector<Lit> &lits)
{
  for(int i = lits.size() - 1 ; i >= 0 ; i--)
    {
      Lit l = lits[i];      
      for(auto &idxCl : occList[l.intern()])
        {
          nbSat[idxCl]--;
          assert(!nbSat[idxCl]);
          
          for(auto &ll : clauses[idxCl])
          {
            if(currentValue[ll.var()] == l_Undef)
              occList[ll.intern()].push_back(idxCl);
          }
        }

      for(auto &idxCl : occList[(~l).intern()]) nbUnsat[idxCl]--;
      currentValue[l.var()] = l_Undef;
    }

  popPreviousClauseSet();
}// postUpdate


/**
   Remove a value from a vector.
*/
void SpecManagerCnfDyn::removeIdxFromOccList(std::vector<int> &o, int idx)
{
  assert(o.size());

  for(unsigned i = 0 ; i<o.size() ; i++)
  {
    if(o[i] == idx)
    {
      o[i] = o.back();
      o.pop_back();
      return;
    }
  }
  assert(0);
}// removeIdxFromOccList

} // d4
