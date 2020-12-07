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
  
}// SpecManagerCnfDyn


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
      m_currentValue[l.var()] = l.sign() ? l_False : l_True;

      for(unsigned i = 0 ; i<2 ; i++)
      {
        std::vector<std::vector<int > > &occList = (i) ? m_occListBin :
                                                   m_occListNotBin;
        for(auto &idxCl : occList[l.intern()])
        {
          m_infoClauses[idxCl].nbSat++;
          for(auto &ll : m_clauses[idxCl])
            if(m_currentValue[ll.var()] == l_Undef)
              removeIdxFromOccList(occList[ll.intern()], idxCl);            
        }
      
        for(auto &idxCl : occList[(~l).intern()])
        {
          m_infoClauses[idxCl].nbUnsat++;
          if(m_infoClauses[idxCl].watcher == ~l) reviewWatcher.push_back(idxCl);
        }
      }
    }

  // we search another non assigned literal if requiered
  for(auto &idxCl : reviewWatcher)
  {
    if(m_infoClauses[idxCl].nbSat) continue;

    for(auto &l : m_clauses[idxCl])
    {
      if(m_currentValue[l.var()] == l_Undef)
      {
        m_infoClauses[idxCl].watcher = l;
        break;
      }
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

    for(unsigned i = 0 ; i<2 ; i++)      
    {
      std::vector<std::vector<int > > &occList = (i) ? m_occListBin :
                                                 m_occListNotBin;      
      for(auto &idxCl : occList[l.intern()])
      {
        m_infoClauses[idxCl].nbSat--;
        assert(!m_infoClauses[idxCl].nbSat);
          
        for(auto &ll : m_clauses[idxCl])
        {
          if(m_currentValue[ll.var()] == l_Undef)
            occList[ll.intern()].push_back(idxCl);
        }
      }
    
      for(auto &idxCl : occList[(~l).intern()]) m_infoClauses[idxCl].nbUnsat--;
    }
    
    m_currentValue[l.var()] = l_Undef;
  }
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
