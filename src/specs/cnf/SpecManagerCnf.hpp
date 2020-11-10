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
class SpecManagerCnf : public SpecManager
{
 protected:
  std::vector<std::vector<Lit> > clauses;
  unsigned nbVar, maxSizeClause;
  std::vector<lbool> currentValue;
  std::vector<int> nbUnsat;
  std::vector<int> nbSat;
  std::vector<Lit> watcher;

  std::vector<int> currentIdx;
  int currentSize;
  std::vector<int> stackSize;
  std::vector<bool> inCurrentComponent;
  std::vector< std::vector<int> > occList;
  
  // to manage the connected component
  std::vector<int> mustUnMark;
  std::vector<Var> tmpVecVar;
  std::vector<int> idxComponent;
  std::vector<bool> tmpMark, markView;

  inline void resetUnMark()
  {
    for(auto &idx : mustUnMark) markView[idx] = false;
    mustUnMark.resize(0);
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

  int getInitSize(int i){return clauses[i].size() - nbUnsat[i];}
  int getCurrentSize(int i){return clauses[i].size() - nbUnsat[i];}

  bool isSatisfiedClause(unsigned idx);
  bool isSatisfiedClause(std::vector<Lit> &c);
  bool isNotSatisfiedClauseAndInComponent(int idx,
                                          std::vector<bool> &inCurrentComponent);
  
  void getCurrentClauses(std::vector<int> &idxClauses,
                         std::vector<bool> &inCurrentComponent);
  
  void updateCurrentFormula(std::vector<Var> &component);
  void popPreviousFormula();

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
  inline int getNbClause(Lit l){return occList[l.intern()].size();}
  inline int getNbClause(){return clauses.size();}
  inline int getNbVariable(){return nbVar;}
  inline int getMaxSizeClause(){return maxSizeClause;}

    virtual inline int getSumSizeClauses()
  {
    int sum = 0;
    for(auto &cl : clauses) sum += cl.size();
    return sum;
  }// getSumSizeClauses

  inline int getNbBinaryClause(Lit l)
  {
    int nbBin = 0;
    for(auto &idx : occList[l.intern()])
      if(clauses[idx].size() - nbUnsat[idx] == 2) nbBin++;
    return nbBin;
  } // getNbBinaryClause


  // about the clauses.
  inline int getNbUnsat(int idx){return nbUnsat[idx];}
  inline int getSize(int idx){return clauses[idx].size() - nbUnsat[idx];}

  inline std::vector<Lit> &getClause(int idx)
  {
    assert((unsigned) idx < clauses.size());
    return clauses[idx];
  }

  
  // about the assignment.
  inline bool varIsAssigned(Var v) override {return currentValue[v] != l_Undef;}
  inline bool litIsAssigned(Lit l) override {return currentValue[l.var()] != l_Undef;}
  inline bool litIsAssignedToTrue(Lit l) override
  {
    if(l.sign()) return currentValue[l.var()] == l_False;
    else return currentValue[l.var()] == l_True;
  }


  // about the occurrence list.
  inline const std::vector< std::vector<int> > &getOccurrenceList(){return occList;}
  inline int getNbOccurrence(Lit l){return getNbClause(l);}
  
  inline std::vector<int> &getVecIdxClause(Lit l)
  {
    assert(l.intern() < occList.size());
    return occList[l.intern()];
  }


  inline void showOccurenceList(std::ostream &out)
  {
    for(unsigned i = 0 ; i<occList.size() ; i++)
    {
      if(!occList[i].size()) continue;
      out << ((i&1) ? "-" : "") << (i>>1) << " --> [ ";
      for(auto l : occList[i]) out << l << " ";
      out << " ]\n";
    }
  }
};
} // d4
