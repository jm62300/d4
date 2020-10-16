#ifndef d4_problem_cnf_DynamicOccurrenceManager_hpp
#define d4_problem_cnf_DynamicOccurrenceManager_hpp

#include "CnfOccurrenceManager.hpp"

namespace d4
{
class DynamicOccurrenceManager : public CnfOccurrenceManager
{
private:
  void initClauses(std::vector<std::vector<Lit> > &clauses);

public:
  DynamicOccurrenceManager(int nbClause, int nbVar, int maxClauseSize);
  DynamicOccurrenceManager(std::vector<std::vector<Lit> > &clauses, int nbVar);

  void preUpdate(std::vector<Lit> &lits);
  void postUpdate(std::vector<Lit> &lits);
  void debug(Solver &s);
  void removeIdxFromOccList(std::vector<int> &o, int idx);

  // we cannot use this function here
  inline void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units){assert(0);}
  void initFormula(std::vector<std::vector<Lit> > &_clauses);
};
}

#endif
