#ifndef d4_src_solvers_WrapperMnisat_hpp
#define d4_src_solvers_WrapperMnisat_hpp

#include "WrapperSolver.hpp"
#include "../problem/ProblemTypes.hpp"
#include "../problem/ProblemManager.hpp"
#include "minisat/Solver.hpp"

namespace d4
{
class WrapperMinisat : public WrapperSolver
{
 private:
  minisat::Solver s;
  
 public:
  void initSolver(ProblemManager &p);
};
}

#endif
