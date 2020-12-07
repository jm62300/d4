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

#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

#include "../SpecManager.hpp"


namespace d4
{
struct SpecClauseInfo
{
  unsigned nbSat;
  unsigned nbUnsat;
  Lit watcher;

  SpecClauseInfo() : nbSat(0), nbUnsat(0), watcher(lit_Undef) {;}
};

class SpecManagerCnf : public SpecManager
{
 protected:
  std::vector<std::vector<Lit> > m_clauses;
  std::vector<int> m_clausesNotBin;
  unsigned m_nbVar, m_maxSizeClause;
  std::vector<lbool> m_currentValue;
  std::vector<SpecClauseInfo> m_infoClauses;

  std::vector<bool> m_inCurrentComponent;
  std::vector< std::vector<int> > m_occListBin;
  std::vector< std::vector<int> > m_occListNotBin;
  
  // to manage the connected component
  std::vector<int> m_mustUnMark;
  std::vector<Var> m_tmpVecVar;
  std::vector<int> m_idxComponent;
  std::vector<bool> m_markView;

  inline void resetUnMark()
  {
    for(auto &idx : m_mustUnMark) m_markView[idx] = false;
    m_mustUnMark.resize(0);
  }// resetUnMark
  
  void connectedToLit(Lit l, std::vector<int> &v,
                      std::vector<Var> &varComponent,
                      int nbComponent);

 public:
  SpecManagerCnf(int nbClause, int nbVar, int maxClauseSize);
  SpecManagerCnf(ProblemManager &p);

  int computeConnectedComponent(std::vector< std::vector<Var> > &varConnected,
                                std::vector<Var> &setOfVar,
                                std::vector<Var> &freeVar,
                                std::vector<Var> &notFreeVar) override;

  void initFormula(ProblemManager &p) override;  
  void showFormula(std::ostream &out) override;
  void showCurrentFormula(std::ostream &out) override;

  int getInitSize(int i){return m_clauses[i].size() - m_infoClauses[i].nbUnsat;}
  int getCurrentSize(int i){return m_clauses[i].size() - m_infoClauses[i].nbUnsat;}

  bool isSatisfiedClause(unsigned idx);
  bool isSatisfiedClause(std::vector<Lit> &c);
  bool isNotSatisfiedClauseAndInComponent(int idx,
                                          std::vector<bool> &m_inCurrentComponent);
  
  void getCurrentClauses(std::vector<unsigned> &idxClauses,
                         std::vector<Var> &component);

  void getCurrentClausesNotBin(std::vector<unsigned> &idxClauses,
                               std::vector<Var> &component);
  
  
  // inline functions.
  // about the CNF.
  inline int getNbBinaryClause(Var v)
  {
    return getNbBinaryClause(Lit::makeLitFalse(v))
        + getNbBinaryClause(Lit::makeLitTrue(v));
  }
  inline int getNbNotBinaryClause(Lit l){return getNbClause(l) - getNbBinaryClause(l);}
  inline int getNbNotBinaryClause(Var v){return getNbClause(v) - getNbBinaryClause(v);}
  inline int getNbClause(Var v)
  {
    return getNbClause(Lit::makeLitFalse(v)) + getNbClause(Lit::makeLitTrue(v));
  }

  inline int getNbClause(Lit l)
  {
    return m_occListBin[l.intern()].size() + m_occListNotBin[l.intern()].size();
  }
  
  inline int getNbClause(){return m_clauses.size();}
  inline int getNbVariable(){return m_nbVar;}
  inline int getMaxSizeClause(){return m_maxSizeClause;}

  virtual inline int getSumSizeClauses()
  {
    int sum = 0;
    for(auto &cl : m_clauses) sum += cl.size();
    return sum;
  }// getSumSizeClauses

  inline int getNbBinaryClause(Lit l)
  {
    int nbBin = m_occListBin[l.intern()].size();
    for(auto &idx : m_occListNotBin[l.intern()])
      if(m_clauses[idx].size() - m_infoClauses[idx].nbUnsat == 2) nbBin++;
    return nbBin;
  } // getNbBinaryClause


  // about the clauses.
  inline int getNbUnsat(int idx){return m_infoClauses[idx].nbUnsat;}
  inline int getSize(int idx){return m_clauses[idx].size() - m_infoClauses[idx].nbUnsat;}

  inline std::vector<Lit> &getClause(int idx)
  {
    assert((unsigned) idx < m_clauses.size());
    return m_clauses[idx];
  }

  
  // about the assignment.
  inline bool varIsAssigned(Var v) override {return m_currentValue[v] != l_Undef;}
  inline bool litIsAssigned(Lit l) override {return m_currentValue[l.var()] != l_Undef;}
  inline bool litIsAssignedToTrue(Lit l) override
  {
    if(l.sign()) return m_currentValue[l.var()] == l_False;
    else return m_currentValue[l.var()] == l_True;
  }


  // about the occurrence list.
  inline const std::vector< std::vector<int> > &getOccurrenceListBin(){return m_occListBin;}
  inline const std::vector< std::vector<int> > &getOccurrenceListNotBin(){return m_occListNotBin;}
  
  inline int getNbOccurrence(Lit l){return getNbClause(l);}
  
  inline std::vector<int> &getVecIdxClauseBin(Lit l)
  {
    assert(l.intern() < m_occListBin.size());
    return m_occListBin[l.intern()];
  }

  inline std::vector<int> &getVecIdxClauseNotBin(Lit l)
  {
    assert(l.intern() < m_occListNotBin.size());
    return m_occListNotBin[l.intern()];
  }


  inline void showOccurenceList(std::ostream &out)
  {
    for(unsigned i = 0 ; i<m_occListBin.size() ; i++)
    {
      if(!m_occListBin[i].size()) continue;
      out << ((i&1) ? "-" : "") << (i>>1) << " --> [ ";
      for(auto l : m_occListBin[i]) out << l << " ";
      out << " ]\n";
    }

    for(unsigned i = 0 ; i<m_occListNotBin.size() ; i++)
    {
      if(!m_occListNotBin[i].size()) continue;
      out << ((i&1) ? "-" : "") << (i>>1) << " --> [ ";
      for(auto l : m_occListNotBin[i]) out << l << " ";
      out << " ]\n";
    }
  }
};
} // d4
