#ifndef d4_src_problem_OccurrenceManager_hpp
#define d4_src_problem_OccurrenceManager_hpp

#include <vector>
#include "ProblemTypes.hpp"
#include "ProblemManager.hpp"

namespace d4
{
class OccurrenceManager
{
public:
  virtual ~OccurrenceManager(){}
  virtual bool litIsAssigned(Lit l) = 0;
  virtual bool litIsAssignedToTrue(Lit l) = 0;
  virtual bool varIsAssigned(Var v) = 0;
  virtual int computeConnectedComponent(std::vector<std::vector<Var> > &varConnected, std::vector<Var> &setOfVar,
                                        std::vector<Var> &freeVar, std::vector<Var> &notFreeVar) = 0;
  virtual void preUpdate(std::vector<Lit> &lits) = 0;
  virtual void postUpdate(std::vector<Lit> &lits) = 0;
  virtual void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units) = 0;
  virtual void showFormula(std::ostream &out) = 0;
  virtual void initFormula(ProblemManager &p) = 0;
  virtual bool byPass(int mode, int idx) = 0;
};
}
#endif
