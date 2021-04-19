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

#include "../WrapperSolver.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"

#include "3rdParty/glucose-3.0/core/Solver.h"

namespace d4 {
class WrapperGlucose : public WrapperSolver {
private:
  Glucose::Solver s;
  Glucose::vec<Glucose::Var> m_setOfVar_m;

  std::vector<Lit> m_assumption;
  std::vector<lbool> m_model;
  bool m_activeModel;
  bool m_needModel;

protected:
  using WrapperSolver::m_isInAssumption;

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
  std::vector<Lit> &getAssumption();
  void pushAssumption(Lit l);
  void popAssumption(unsigned count);
  void displayAssumption(std::ostream &out);
  void setNeedModel(bool b);
  void showTrail();
  std::vector<lbool> &getModel();
  void getUnits(std::vector<Lit> &units);

  double getActivity(Var v);
  double getCountConflict(Var v);
};
} // namespace d4