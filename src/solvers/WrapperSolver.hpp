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
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#pragma once

#include <span>
#include <vector>

#include "3rdParty/cadical/src/cadical.hpp"
#include "ActivityManager.hpp"
#include "OptionSolver.hpp"
#include "PolarityManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
class WrapperSolver : public ActivityManager, public PolarityManager {
 protected:
  std::vector<char> m_isInAssumption;
  std::vector<Lit> m_assumption;
  std::vector<lbool> m_model;
  bool m_activeModel;
  bool m_needModel;

  CaDiCaL::Solver m_cadical;
  std::vector<std::vector<int>> m_initClauses;
  unsigned m_nbInitVar = 0;
  unsigned m_initBudget = 500;
  unsigned m_minLimitVar = 50;
  unsigned m_learntFactor = 1;
  unsigned m_cadicalRedundantFactor = 1;

  void initCadical(unsigned nbVar);
  void rebuildCadical();
  virtual void onCadicalSat(std::span<const Var> setOfVar) {}

 public:
  static WrapperSolver* makeWrapperSolver(const OptionSolver& name,
                                          const ProblemManager& p,
                                          std::ostream& out);

  virtual ~WrapperSolver() {}
  virtual void initSolver(const ProblemManager& p) = 0;
  bool solve(std::span<const Var> setOfVar, std::vector<Lit>& units);
  virtual lbool runSolver(std::span<const Var> setOfVar) = 0;

  virtual lbool solveLimited(std::span<const Var> setOfVar,
                             unsigned nbConflict) = 0;

  virtual void uncheckedEnqueue(Lit l) = 0;
  virtual void restart() = 0;
  virtual void setAssumption(const std::vector<Lit>& assums) = 0;
  virtual std::vector<Lit>& getAssumption() = 0;
  virtual void pushAssumption(Lit l) = 0;
  virtual void popAssumption(unsigned count = 1) = 0;
  virtual void displayAssumption(std::ostream& out) = 0;
  virtual bool varIsAssigned(Var v) = 0;
  virtual void setNeedModel(bool b) = 0;
  virtual void showTrail() = 0;
  virtual std::vector<lbool>& getModel() = 0;
  virtual lbool getModelVar(Var v) = 0;
  virtual void getUnits(std::vector<Lit>& units) = 0;
  virtual unsigned getNbConflict() = 0;
  virtual void setReversePolarity(bool value) = 0;
  virtual bool propagateAssumption() = 0;
  virtual bool isUnsat() = 0;

  virtual bool decideAndComputeUnit(Lit l, std::vector<Lit>& units) = 0;

  virtual void whichAreUnits(std::span<const Var> component,
                             std::vector<Lit>& units) = 0;

  inline bool getActiveModel() { return m_activeModel; }
  inline bool getNeedModel() { return m_needModel; }
  unsigned sizeAssumption() { return getAssumption().size(); }

  void configure(const OptionSolver& opts);
  bool warmStart(int iteration, int sizeQuery, std::vector<Var>& setOfVar,
                 std::ostream& out);

  virtual void getCore() = 0;
  virtual void getLastIUP(Lit l) = 0;

  virtual void cleanLearntClauses() = 0;
  virtual unsigned getNbLearntClauses() = 0;

  inline bool isInAssumption(Lit l) {
    return m_isInAssumption[l.var()] == 1 + l.sign();
  }

  inline bool isInAssumption(Var v) { return m_isInAssumption[v]; }

  inline void resetAssumption() { popAssumption(getAssumption().size()); }

  bool getPolarity(Var v) override {
    if (!m_activeModel) return false;
    return m_model[v];
  }
};
}  // namespace d4
