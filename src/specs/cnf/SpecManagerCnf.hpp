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

#ifndef d4_src_problem_cnf_SpecManagerCnf_hpp
#define d4_src_problem_cnf_SpecManagerCnf_hpp

#include <problem/ProblemManager.hpp>
#include <problem/cnf/ProblemManagerCnf.hpp>

#include "../SpecManager.hpp"


namespace d4
{
using namespace std;
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
  
  inline void showOccurenceList(std::ostream &out)
  {
    for(unsigned i = 0 ; i<occList.size() ; i++)
    {
      out << ((i&1) ? "-" : "") << ((i>>1) + 1) << " ";
      for(auto l : occList[i]) out << l << " ";
      out << "\n";
    }
  }  

 protected:
  // to manage the connected component
  std::vector<int> mustUnMark;
  std::vector<Var> tmpVecVar;
  std::vector<int> idxComponent;
  std::vector<bool> tmpMark, markView;

  inline void resetUnMark()
  {
    for(auto idx : mustUnMark) markView[idx] = false;
    mustUnMark.resize(0);
  }// resetUnMark

  int connectedToLit(Lit l, std::vector<int> &v, std::vector<Var> &varComponent,
                     int nbComponent);

 public:
  SpecManagerCnf(int nbClause, int nbVar, int maxClauseSize);
  SpecManagerCnf(ProblemManager &p);

  int computeConnectedComponent(std::vector< std::vector<Var> > &varConnected,
                                std::vector<Var> &setOfVar,
                                std::vector<Var> &freeVar,
                                std::vector<Var> &notFreeVar) override;

  void initFormula(ProblemManager &p) override;

  inline int getNbBinaryClause(Lit l)
  {
    int nbBin = 0;
    for(auto &idx : occList[l.intern()])
      if(clauses[idx].size() - nbUnsat[idx] == 2) nbBin++;
    return nbBin;
  } // getNbBinaryClause
  
  inline void showOccList(std::ostream &out)
  {
    unsigned i = 0;
    for(auto list : occList)
    {
      if(!list.size()) continue;

      out << ((i&1) ? "-" : "") << (i>>1);
      for(auto &idx : list) out << idx << " ";
      out << "\n";
      i++;
    }
  } // showOccList

  
  void showFormula(std::ostream &out) override;
  void showCurrentFormula(std::ostream &out) override;

  int getInitSize(int i){return clauses[i].size() - nbUnsat[i];}
  int getCurrentSize(int i){return clauses[i].size() - nbUnsat[i];}

  bool isSatisfiedClause(unsigned idx);
  bool isSatisfiedClause(std::vector<Lit> &c);
  bool isNotSatisfiedClauseAndInComponent(int idx, std::vector<bool> &inCurrentComponent);
  void getCurrentClauses(std::vector<int> &idxClauses, std::vector<bool> &inCurrentComponent);
  void updateCurrentClauseSet(std::vector<Var> &component);
  void popPreviousClauseSet();

  
  
  inline const std::vector< std::vector<int> > &getOccurrenceList(){return occList;}
  inline int getNbBinaryClause(Var v)
  {
    return getNbBinaryClause(Lit(v, false)) + getNbBinaryClause(Lit(v, true));
  }
  inline int getNbNotBinaryClause(Lit l){return getNbClause(l) - getNbBinaryClause(l);}
  inline int getNbNotBinaryClause(Var v){return getNbClause(v) - getNbBinaryClause(v);}
  inline int getNbClause(Var v){return getNbClause(Lit(v, false)) + getNbClause(Lit(v, true));}
  inline int getNbClause(Lit l){return occList[l.intern()].size();}
  inline int getNbClause(){return clauses.size();}
  inline std::vector<int> &getVecIdxClause(Lit l){return occList[l.intern()];}
  inline std::vector<Lit> &getClause(int idx){return clauses[idx];}
  inline int getNbUnsat(int idx){return nbUnsat[idx];}
  inline int getNbVariable(){return nbVar;}

  inline bool varIsAssigned(Var v) override {return currentValue[v] != l_Undef;}
  inline bool litIsAssigned(Lit l) override {return currentValue[l.var()] != l_Undef;}
  inline bool litIsAssignedToTrue(Lit l) override
  {
    if(l.sign()) return currentValue[l.var()] == l_False;
    else return currentValue[l.var()] == l_True;
  }  

  inline int getMaxSizeClause(){return maxSizeClause;}
  inline int getNbOccurrence(Lit l){return getNbClause(l);}

  virtual inline int getSumSizeClauses()
  {
    int sum = 0;
    for(auto &cl : clauses) sum += cl.size();
    return sum;
  }// getSumSizeClauses
};

} // d4

#endif
