#ifndef MODELCOUNTER_DYNAMIC_OCCURRENCE_MANAGER
#define MODELCOUNTER_DYNAMIC_OCCURRENCE_MANAGER

#include "../utils/System.hh"
#include "../utils/SolverTypes.hh"
#include "../utils/Solver.hh"
#include "../mtl/Sort.hh"
#include "../mtl/Vec.hh"
#include "../mtl/Heap.hh"
#include "../mtl/Alg.hh"
#include "../DAG/DAG.hh"

#include "../manager/CnfOccurrenceManager.hh"

class DynamicOccurrenceManager : public CnfOccurrenceManager
{
private:
  void initClauses(vec<vec<Lit> > &clauses);

public:
  DynamicOccurrenceManager(int nbClause, int nbVar, int maxClauseSize);
  DynamicOccurrenceManager(vec<vec<Lit> > &clauses, int nbVar);

  void preUpdate(vec<Lit> &lits);
  void postUpdate(vec<Lit> &lits);
  void debug(Solver &s);
  void removeIdxFromOccList(vec<int> &o, int idx);

  // we cannot use this function here
  inline void initialize(vec<Var> &setOfVar, vec<Lit> &units){assert(0);}
  void initFormula(vec<vec<Lit> > &_clauses);
};

#endif
