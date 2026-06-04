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

#include "WrapperGlucose.hpp"

#include <iostream>
#include <typeinfo>
#include <vector>

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "3rdParty/glucose-3.0/core/SolverTypes.h"
#include "3rdParty/glucose-3.0/mtl/Vec.h"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
/**
   This function initializes the SAT solver with a given problem.  Warning: we
   suppose that p is a CNF, otherwise a bad_cast exception is threw.

   @param[in] p, the problem we want to link with the SAT solver.
 */
void WrapperGlucose::initSolver(const ProblemManager& p) {
  std::cout << "c [GLUCOSE SOLVER] Init phase\n";
  // say to the solver we have pcnf.getNbVar() variables.
  while ((unsigned)m_solver.nVars() <= p.getNbVar()) m_solver.newVar();
  m_model.resize(p.getNbVar() + 1, l_Undef);

  cadical.set("inprocessing", 0);
  cadical.set("reduceinit", 1000);  // start reducing sooner
  cadical.set("reducefactor", 10);  // reduce more aggressively
  cadical.set("reducetier1glue", 1);

  cadical.declare_more_variables(p.getNbVar() + 1);

  m_startClock = std::clock();
  m_nbInitVar = p.getNbVar();
  m_initClauses.reserve(p.getGates().size());

  // get the clauses.
  for (auto& gate : p.getGates()) {
    assert(gate.gateType == BcGateType::CLAUSE);
    Glucose::vec<Glucose::Lit> lits;
    std::vector<int> cl;

    for (auto& l : gate.input) {
      cl.push_back(l.human());
      cadical.add(l.human());
      lits.push(Glucose::mkLit(l.var(), l.sign()));
    }

    m_initClauses.push_back(cl);
    cadical.add(0);
    m_solver.addClause(lits);
  }

  m_solver.setConfBudget(m_initBudget);
  m_activeModel = false;
  m_needModel = false;
  setNeedModel(m_needModel);
  m_isInAssumption.resize(p.getNbVar() + 1, 0);

}  // initSolver

/**
   Call the SAT solver and return its result.

   @param[in] setOfvar, the variables we focus the solving process.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperGlucose::solve(std::span<const Var> setOfVar,
                           std::vector<Lit>& units) {
  if (m_activeModel && m_needModel) {
    whichAreUnits(setOfVar, units);
    return true;
  }

  if (cadical.redundant() > m_initClauses.size()) rebuildCadical();
  if (m_solver.learnts.size() > m_initClauses.size()) m_solver.removeLearnt();

  m_setOfVar_m.setSize(0);
  for (auto& v : setOfVar) m_setOfVar_m.push(v);
  m_solver.setConfBudget(setOfVar.size() < m_minLimitVar ? -1 : m_initBudget);
  m_solver.rebuildWithConnectedComponent(m_setOfVar_m);

  std::clock_t glucoseStart = std::clock();
  lbool resGlucose = solveLimited(
      setOfVar, setOfVar.size() < m_minLimitVar ? -1 : m_initBudget);

  m_glucoseTime += (double)(std::clock() - glucoseStart) / CLOCKS_PER_SEC;
  if (setOfVar.size() >= m_minLimitVar) {
    ++m_glucoseCalls;
    ++m_totalCalls;
  }

  if (m_totalCalls % 100 == 0) {
    double totalTime = (double)(std::clock() - m_startClock) / CLOCKS_PER_SEC;
    double solverTime = m_glucoseTime + m_cadicalTime;
    double solverPct = totalTime > 0.0 ? 100.0 * solverTime / totalTime : 0.0;
    int64_t cadicalConflicts = cadical.get_statistic_value("conflicts");
    int64_t totalConflicts = (int64_t)m_solver.conflicts + cadicalConflicts;
    std::cout << m_solver.learnts.size() << " learnt clause for glucose\n";
    std::cout << "c [SOLVER] calls=" << m_totalCalls
              << " glucose=" << m_glucoseCalls << " t=" << m_glucoseTime << "s"
              << " cadical=" << m_cadicalCalls << " t=" << m_cadicalTime << "s"
              << " total-cpu=" << totalTime << "s"
              << " solver%=" << solverPct << "%"
              << " conflicts=" << totalConflicts << " (g=" << m_solver.conflicts
              << " c=" << cadicalConflicts << ")\n";
  }

  if (resGlucose == l_False) return false;
  m_activeModel = resGlucose == l_True;

  if (!m_activeModel) {
    assert(m_solver.decisionLevel() == m_assumption.size());
    if (m_initBudget > 1 && m_glucoseCalls > 100 &&
        ((double)m_cadicalCalls / (double)m_totalCalls) > 0.9) {
      std::cout << (double)m_cadicalCalls / (double)m_glucoseCalls
                << " dactivate glucose\n";
      m_initBudget = 1;
    }

    cadical.reset_assumptions();
    for (auto& l : m_assumption) cadical.assume(l.human());

    std::clock_t cadicalStart = std::clock();
    int satCallRes = cadical.solve();

    double current = (double)(std::clock() - cadicalStart) / CLOCKS_PER_SEC;
    m_cadicalTime += current;
    ++m_cadicalCalls;
    m_activeModel = satCallRes == 10;

    if (!m_activeModel) return false;

    // SAT: propagate some information
    for (auto v : setOfVar) m_model[v] = cadical.val(v) > 0 ? l_True : l_False;
    cadical.reset_assumptions();
    for (auto& l : m_assumption) cadical.assume(l.human());

    std::vector<int> implicants;
    int res = cadical.propagate();

    if (res != 20) {  // res == 0, then INCONCLUSIVE, else SAT by BCP
      cadical.implied(implicants);
      if (implicants.size()) {
        Glucose::vec<Glucose::Lit> learnt;
        learnt.push();
        for (auto& l : m_assumption)
          learnt.push(Glucose::mkLit(l.var(), l.sign()));

        for (auto& l : implicants)
          if (!m_solver.isAssigned(std::abs(l))) {
            learnt[0] = Glucose::mkLit(std::abs(l), l < 0);
            m_solver.addLearntClauseWithPropagation(learnt);
          }

        m_solver.propagate();
      }
    }
  } else {
    // it is SAT!
    for (auto v : setOfVar) {
      assert(m_solver.model[v] != Glucose::l_Undef);
      m_model[v] = m_solver.model[v] == Glucose::l_True ? l_True : l_False;
    }
  }

  for (auto v : setOfVar)
    if (m_solver.isAssigned(v))
      units.push_back(Lit::makeLit(v, sign(m_solver.litAssigned(v))));

  assert(!m_activeModel || m_solver.decisionLevel() == m_assumption.size());
  assert(!m_assumption.size() || units.size());  // at least the assumption!
  return m_activeModel;
}  // solve

/**
 * @brief Strong assumption in the sense we push the literal on the stack.
 *
 * @param l the literal we want to push.
 */
void WrapperGlucose::uncheckedEnqueue(Lit l) {
  m_solver.uncheckedEnqueue(Glucose::mkLit(l.var(), l.sign()));
}  // uncheckedEnqueue

/**
 * @brief Enforce the unit propagation of all the assumption literals.
 *
 * @return true if we did not reach a conflict, false otherwise.
 */
bool WrapperGlucose::propagateAssumption() {
  return m_solver.propagateAssumption();
}  // propagateAssumption

/**
   Call the SAT solver and return its result.

   \return true if the problem is SAT, false otherwise.
 */
lbool WrapperGlucose::solveLimited(std::span<const Var> setOfVar,
                                   unsigned nbConflict) {
  m_setOfVar_m.setSize(0);
  for (auto& v : setOfVar) m_setOfVar_m.push(v);
  m_solver.rebuildWithConnectedComponent(m_setOfVar_m);
  m_solver.setConfBudget(nbConflict);
  Glucose::lbool res = m_solver.solve_(false);

  if (res == Glucose::l_Undef) return l_Undef;
  return res == Glucose::l_True ? l_True : l_False;
}  // solve

/**
   An accessor on the activity of a variable.

   @param[in] v, the variable we want the activity.

   \return the activity of v.
 */
double WrapperGlucose::getActivity(Var v) {
  return m_solver.activity[v];
}  // getActivity

/**
   Return the number of times the variable v occurs in a conflict.

   @param[in] v, the variable we want the activity.

   \return the number of times v occurs in a conflict.
 */
double WrapperGlucose::getCountConflict(Var v) {
  return m_solver.scoreActivity[v];
}  // getCountConflict

/**
 * @brief decayCountConflict implementation.
 *
 */
void WrapperGlucose::decayCountConflict() {
  for (unsigned i = 0; i < m_solver.scoreActivity.size(); i++)
    m_solver.scoreActivity[i] = m_solver.scoreActivity[i] / 2;
}  // decayCountConflict

/**
 * @brief Set the count conflict in the solver.
 * @param[in] v is the variable we want to set the counter of  conflicts.
 * @param[in] count is the count we want to assign.
 */
void WrapperGlucose::setCountConflict(Var v, double count) {
  m_solver.scoreActivity[v] = count;
}  // setCountConflict

/**
   Print out the trail on the standard output.
 */
void WrapperGlucose::showTrail() { m_solver.showTrail(); }  // showTrail

/**
 * @brief Set the reverse polarity flag to the solver.
 *
 * @param value is the value we want to assign.
 */
void WrapperGlucose::setReversePolarity(bool value) {
  m_solver.reversePolarity = value;
}  // setReversePolarity

/**
   Collect the unit literal from the affectation of the literal l to the
   formula.

   @param[in] l, the literal we want to branch on.
   @param[out] units, the unit literals

   \return true if assign l and propagate does not give a conflict, false
   otherwise.
 */
bool WrapperGlucose::decideAndComputeUnit(Lit l, std::vector<Lit>& units) {
  Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());
  if (varIsAssigned(l.var())) {
    if (m_solver.litAssigned(l.var()) != ml) return false;
    units.push_back(l);
    return true;
  }

  int posTrail = (m_solver.trail).size();
  m_solver.newDecisionLevel();
  m_solver.uncheckedEnqueue(ml);
  Glucose::CRef confl = m_solver.propagate();

  if (confl != Glucose::CRef_Undef)  // unit literal
  {
    int bt;
    Glucose::vec<Glucose::Lit> learnt_clause;
    m_solver.analyzeLastUIP(confl, learnt_clause, bt);
    m_solver.cancelUntil(m_solver.decisionLevel() - 1);
    assert(learnt_clause[0] == Glucose::mkLit(l.var(), !l.sign()));
    m_solver.insertClauseAndPropagate(learnt_clause);
    return false;
  }

  for (int j = posTrail; j < m_solver.trail.size(); j++)
    units.push_back(
        Lit::makeLit(var(m_solver.trail[j]), sign(m_solver.trail[j])));
  m_solver.cancelUntil(m_solver.decisionLevel() - 1);
  return true;
}  // decideAndComputeUnit

/**
   Fill the vector units with the literal l that are units such that l.var() is
   in component.

   @param[in] component, the set of variables we search for.
   @param[out] units, the place where we store the literals found.
 */
void WrapperGlucose::whichAreUnits(std::span<const Var> component,
                                   std::vector<Lit>& units) {
  assert(m_solver.decisionLevel() == m_assumption.size());
  for (auto& v : component) {
    if (!m_solver.isAssigned(v)) continue;

    Glucose::Lit l = m_solver.litAssigned(v);
    units.push_back(Lit::makeLit(var(l), sign(l)));
  }
}  // whichAreUnits

/**
 * @brief Get the list of unit literals that are in the trail (we suppose that
 * the decision level is zero).
 *
 * @param[out] units is the list of unit literals.
 */
void WrapperGlucose::getUnits(std::vector<Lit>& units) {
  for (int i = 0; i < m_solver.trail.size(); i++) {
    Glucose::Lit l = m_solver.trail[i];
    units.push_back(Lit::makeLit(var(l), sign(l)));
  }
}  // getUnits

/**
   Check out if the given variable is assigned or not by the solver.

   @param[in] v, the variable we search for.

   \return true if the variable is assigned, false otherwise.
 */
bool WrapperGlucose::varIsAssigned(Var v) {
  return m_solver.isAssigned(v);
}  // varIsAssigned

/**
   Restart the solver.
 */
void WrapperGlucose::restart() { m_solver.cancelUntil(0); }  // restart

/**
   Transfer to the solver the fact we have a set of assumption variables we want
   to consider.

   @param[in] assums, the set of assumptions
 */
void WrapperGlucose::setAssumption(const std::vector<Lit>& assums) {
  popAssumption(m_assumption.size());
  Glucose::vec<Glucose::Lit>& assumptions = m_solver.assumptions;
  assumptions.clear();
  m_assumption.clear();
  for (auto& l : assums) pushAssumption(l);
}  // setAssumption

/**
   \return the current assumption.

   @param[in] assums, the set of assumptions
 */
std::vector<Lit>& WrapperGlucose::getAssumption() {
  return m_assumption;
}  // getAssumption

/**
   Print out the assumption.

   @param[in] out, the stream where is print the assumption.
 */
void WrapperGlucose::displayAssumption(std::ostream& out) {
  Glucose::vec<Glucose::Lit>& assumptions = m_solver.assumptions;
  for (int i = 0; i < assumptions.size(); i++) {
    Glucose::Lit l = assumptions[i];
    std::cout << (Glucose::sign(l) ? "-" : "") << Glucose::var(l) << " ";
  }
  std::cout << "\n";
}  // displayAssumption

/**
   Ask for the model.

   @param[in] b, a boolean value to ask the solver to get the model.
 */
void WrapperGlucose::setNeedModel(bool b) {
  m_needModel = b;
  m_solver.setNeedModel(b);
}  // setNeedModel

/**
 * @brief Return the model computed by the solver.
 *
 * @return the model's value (lbool).
 */
std::vector<lbool>& WrapperGlucose::getModel() { return m_model; }  // getModel

/**
 * @brief Get the value given by the last computed model.
 *
 * @param v is the variable we want to get the assignment.
 * @return the last value of v.
 */
lbool WrapperGlucose::getModelVar(Var v) {
  return Glucose::toInt(m_solver.model[v]);
}  // getModelVar

/**
   Push a new assumption.

   @param[in] l, the literal we want to push.
 */
void WrapperGlucose::pushAssumption(Lit l) {
  Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());
  m_activeModel = m_activeModel && !m_solver.isAssigned(var(ml));

  (m_solver.assumptions).push(ml);
  m_assumption.push_back(l);
  assert((m_solver.assumptions).size() == m_assumption.size());

  assert(!m_isInAssumption[l.var()]);
  m_isInAssumption[l.var()] = 1 + l.sign();

  if (m_activeModel && m_needModel) {
    m_activeModel = m_model[l.var()] == l.sign();
    if (m_activeModel) {
      assert(m_solver.decisionLevel() == m_solver.assumptions.size() - 1);
      m_solver.newDecisionLevel();
      assert(!m_solver.isAssigned(var(ml)));
      m_solver.uncheckedEnqueue(ml);
      [[maybe_unused]] Glucose::CRef cref = m_solver.propagate();
      assert(cref == Glucose::CRef_Undef);
    }
  }
}  // pushAssumption

/**
   Remove the last assumption and cancelUntil.

   @param[in] count, the number of element we pop.
 */
void WrapperGlucose::popAssumption(unsigned count) {
  for (unsigned i = m_assumption.size() - count; i < m_assumption.size(); i++) {
    assert(m_isInAssumption[m_assumption[i].var()]);
    m_isInAssumption[m_assumption[i].var()] = 0;
  }
  m_assumption.resize(m_assumption.size() - count);
  (m_solver.assumptions).shrink_(count);
  (m_solver.cancelUntil)((m_solver.assumptions).size());
}  // popAssumption
}  // namespace d4