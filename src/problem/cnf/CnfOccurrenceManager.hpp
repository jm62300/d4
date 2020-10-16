#ifndef d4_problem_cnf_CnfOccurrenceManager_hpp
#define d4_problem_cnf_CnfOccurrenceManager_hpp

#include <iostream>
#include <vector>

#include "../OccurrenceManagerInterface.hpp"
#include "../BucketManagerInterface.hpp"

namespace d4
{
class CnfOccurrenceManager : public OccurrenceManagerInterface
{
protected:
  std::vector<std::vector<Lit> > clauses;
  int nbVar, maxSizeClause;
  std::vector<lbool> currentValue;
  std::vector<int> nbUnsat;
  std::vector<int> nbSat;
  std::vector<Lit> watcher;

  std::vector<int> currentIdx;
  int currentSize;
  std::vector<int> stackSize;
  std::vector<bool> inCurrentComponent;

  inline void showOccurenceList()
  {
    for(int i = 0 ; i<occList.size() ; i++)
      {
        printf("%s%d: ", (i&1) ? "-" : "", (i>>1) + 1);
        for(int j = 0 ; j<occList[i].size() ; j++) printf("%d ", occList[i][j]);
        printf("\n");
      }
  }

protected:
  // to manage the connected component
  std::vector<int> mustUnMark;
  std::vector<Var> tmpStd::VectorVar;
  std::vector<int> idxComponent;
  std::vector<bool> tmpMark, markView;

  inline void resetUnMark()
  {
    for(int i = 0 ; i<mustUnMark.size() ; i++) markView[mustUnMark[i]] = false;
    mustUnMark.setSize(0);
  }// resetUnMark

  int connectedToLit(Lit l, std::vector<int> &v, std::vector<Var> &varComponent, int nbComponent);

public:
  CnfOccurrenceManager(int nbClause, int nbVar, int maxClauseSize);
  CnfOccurrenceManager(std::vector<std::vector<Lit> > &clauses, int nbVar);

  int computeConnectedComponent(std::vector< std::vector<Var> > &varConnected, std::vector<Var> &setOfVar, std::vector<Var> &freeVar,
                                std::vector<Var> &notFreeVar);

  inline void initFormula(std::vector<std::vector<Lit> > &_clauses)
  {
    clauses.clear();
    currentIdx.clear();

    for(int i = 0 ; i<_clauses.size() ; i++)
      {
        clauses.push();
        assert(_clauses[i].size());
        _clauses[i].copyTo(clauses.last());
        currentIdx.push(i);
      }

    currentSize = clauses.size();
    stackSize.clear();
    for(int i = 0 ; i<currentValue.size() ; i++) currentValue[i] = l_Undef;
  }// initFormula

  inline int getNbBinaryClause(Lit l)
  {
    int nbBin = 0;

    for(int i = 0 ; i<occList[toInt(l)].size() ; i++)
      {
        int idx = occList[toInt(l)][i];
        if(clauses[idx].size() - nbUnsat[idx] == 2) nbBin++;
      }

    return nbBin;
  }

  inline void showOccList()
  {
    for(int i = 0 ; i<occList.size() ; i++)
      {
        Lit l = mkLit(i>>1, i&1);
        if(!occList[i].size()) continue;

        printf("%d: ", readableLit(l));
        for(int j = 0 ; j<occList[i].size() ; j++) printf("%d ", occList[i][j]);
        printf("\n");
      }
  }


  inline void showFormula()
  {
    printf("Occurrence Managaer: print formula\n");
    for(int i = 0 ; i<clauses.size() ; i++) showListLit(clauses[i]);
  }// showFormula


  inline int getNbBinaryClause(Var v){return getNbBinaryClause(mkLit(v, false)) + getNbBinaryClause(mkLit(v, true));}
  inline int getNbNotBinaryClause(Lit l){return getNbClause(l) - getNbBinaryClause(l);}
  inline int getNbNotBinaryClause(Var v){return getNbClause(v) - getNbBinaryClause(v);}
  inline int getNbClause(Var v){return getNbClause(mkLit(v, false)) + getNbClause(mkLit(v, true));}
  inline int getNbClause(Lit l){return occList[toInt(l)].size();}
  inline int getNbClause(){return clauses.size();}
  inline std::vector<int> &getStd::VectorIdxClause(Lit l){return occList[toInt(l)];}
  inline std::vector<Lit> &getClause(int idx){return clauses[idx];}
  inline int getNbUnsat(int idx){return nbUnsat[idx];}
  inline int getNbVariable(){return nbVar;}

  inline bool litIsAssigned(Lit l){return currentValue[var(l)] != l_Undef;}
  inline bool litIsAssignedToTrue(Lit l)
  {
    if(sign(l)) return currentValue[var(l)] == l_False;
    else return currentValue[var(l)] == l_True;
  }

  inline bool varIsAssigned(Var v){return currentValue[v] != l_Undef;}
  inline int getMaxSizeClause(){return maxSizeClause;}

  virtual inline int getSumSizeClauses()
  {
    int sum = 0;
    for(int i = 0 ; i<clauses.size() ; i++) sum += clauses[i].size();
    return sum;
  }// getSumSizeClauses

  bool isSatisfiedClause(int idx);
  bool isSatisfiedClause(std::vector<Lit> &c);
  bool isNotSatisfiedClauseAndInComponent(int idx, std::vector<bool> &inCurrentComponent);


  // debug part
  void checkCurrentInterpretation(Solver &s);

  inline bool byPass(int mode, int idx)
  {
    if(mode >= NB && clauses[idx].size() <= 2) return true;
    if(mode == NT && !nbUnsat[idx]) return true;
    return false;
  }

  void getCurrentClauses(std::vector<int> &idxClauses, std::vector<bool> &inCurrentComponent);
  void updateCurrentClauseSet(std::vector<Var> &component);
  void popPreviousClauseSet();
};
}

#endif
