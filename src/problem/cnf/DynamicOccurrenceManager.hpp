#ifndef d4_src_problem_cnf_DynamicOccurrenceManager_hpp
#define d4_src_problem_cnf_DynamicOccurrenceManager_hpp

#include <vector>
#include <cassert>

#include "../ProblemTypes.hpp"
#include "../ProblemManager.hpp"

#include "CnfOccurrenceManager.hpp"


namespace d4
{
class DynamicOccurrenceManager : public CnfOccurrenceManager
{
 private:
  void initClauses(std::vector<std::vector<Lit> > &clauses);

 public:
  DynamicOccurrenceManager(int nbClause, int nbVar, int maxClauseSize);
  DynamicOccurrenceManager(ProblemManager &p);

  void preUpdate(std::vector<Lit> &lits);
  void postUpdate(std::vector<Lit> &lits);
  void removeIdxFromOccList(std::vector<int> &o, int idx);

  // we cannot use this function here
  inline void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units){assert(0);}
  void initFormula(ProblemManager &p);
};

}

#endif
