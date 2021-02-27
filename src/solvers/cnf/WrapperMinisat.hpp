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

#ifndef d4_src_solvers_cnf_WrapperMnisat_hpp
#define d4_src_solvers_cnf_WrapperMnisat_hpp

#include "../WrapperSolver.hpp"

#include <src/problem/ProblemTypes.hpp>
#include <src/problem/ProblemManager.hpp>

#include "minisat/Solver.hpp"

namespace d4
{
class WrapperMinisat : public WrapperSolver
{
 private:
  minisat::Solver s;  
  minisat::vec<minisat::Var> m_setOfVar_m;

  bool m_activeModel;
  bool m_needModel;
  
 public:
  void initSolver(ProblemManager &p);
  bool solve(std::vector<Var> &setOfVar);
  bool solve();
  bool varIsAssigned(Var v);
  bool getPolarity(Var v);
  bool decideAndComputeUnit(Lit l, std::vector<Lit> &units);
  
  void getSimplifiedFormula(ProblemManager &p);
  void whichAreUnits(std::vector<Var> &component, std::vector<Lit> &units);
  void restart();
  void setAssumption(std::vector<Lit> &assums);
  void pushAssumption(Lit l);
  void popAssumption();
  void setNeedModel(bool b);
  void showTrail();
  
  double getActivity(Var v);
  double getCountConflict(Var v);

  inline void bumpActivity(Var v)
  {
    s.varDecayActivity();
    s.varBumpActivity(v);
  }
};
}

#endif
