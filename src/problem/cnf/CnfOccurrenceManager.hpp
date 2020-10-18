#ifndef d4_src_problem_cnf_CnfOccurrenceManager_hpp
#define d4_src_problem_cnf_CnfOccurrenceManager_hpp

#include "../ProblemManager.hpp"
#include "../OccurrenceManager.hpp"
#include "ProblemManagerCnf.hpp"

namespace d4
{
using namespace std;

class CnfOccurrenceManager : public OccurrenceManager
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

  int connectedToLit(Lit l, std::vector<int> &v, std::vector<Var> &varComponent, int nbComponent);

 public:
  CnfOccurrenceManager(int nbClause, int nbVar, int maxClauseSize);
  CnfOccurrenceManager(ProblemManager &p);

  int computeConnectedComponent(std::vector< std::vector<Var> > &varConnected,
                                std::vector<Var> &setOfVar, std::vector<Var> &freeVar,
                                std::vector<Var> &notFreeVar) override;

  inline void initFormula(ProblemManager &p) override
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

  
  inline void showFormula(std::ostream &out) override
  {
    printf("Occurrence Managaer: print formula\n");
    for(auto &cl : clauses) showListLit(out, cl);
  }// showFormula

  
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

  virtual inline int getSumSizeClauses()
  {
    int sum = 0;
    for(auto &cl : clauses) sum += cl.size();
    return sum;
  }// getSumSizeClauses

  bool isSatisfiedClause(unsigned idx);
  bool isSatisfiedClause(std::vector<Lit> &c);
  bool isNotSatisfiedClauseAndInComponent(int idx, std::vector<bool> &inCurrentComponent);
  void getCurrentClauses(std::vector<int> &idxClauses, std::vector<bool> &inCurrentComponent);
  void updateCurrentClauseSet(std::vector<Var> &component);
  void popPreviousClauseSet();
};

}

#endif
