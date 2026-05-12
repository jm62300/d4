/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
// #include <bits/stdint-uintn.h>
#include "src/solver/WrapperGlucose.hpp"

#include <iostream>
#include <typeinfo>
#include <vector>

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "3rdParty/glucose-3.0/core/SolverTypes.h"
#include "3rdParty/glucose-3.0/mtl/Vec.h"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {

/**
 * @brief WrapperGlucose::initSolver implementation.
 */
void WrapperGlucose::initSolver(Problem& pcnf) {
  try {
    // force Glucose_bipe to be in incremental mode in order to restart just
    // after the assumptions. s.setIncrementalMode();

    // say to the solver we have pcnf.getNbVar() variables.
    while ((unsigned)s.nVars() <= pcnf.getNbVar()) s.newVar();
    m_model.resize(pcnf.getNbVar() + 1, l_Undef);

    // load the clauses
    std::vector<std::vector<Lit>>& clauses = pcnf.getClauses();
    for (auto& cl : clauses) {
      Glucose::vec<Glucose::Lit> lits;
      for (auto& l : cl) lits.push(Glucose::mkLit(l.var(), l.sign()));
      s.addClause(lits);
    }
  } catch (std::bad_cast& bc) {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }

  m_activeModel = false;
  m_needModel = false;
  setNeedModel(m_needModel);
  m_isInAssumption.resize(pcnf.getNbVar() + 1, 0);
}  // initSolver

/**
 * @brief WrapperGlucose::displayToCnf implementation.
 */
void WrapperGlucose::displayToCnf(std::ostream& out) {
  unsigned limitUnit = s.trail_lim.size() ? s.trail_lim[0] : 0;
  if (!s.decisionLevel()) limitUnit = s.trail.size();

  out << "p cnf " << s.nVars() << " " << s.clauses.size() + limitUnit << "\n";
  for (int i = 0; i < limitUnit; i++) out << litToInt(s.trail[i]) << " 0\n";

  for (int i = 0; i < s.clauses.size(); i++) {
    Glucose::Clause& c = s.ca[s.clauses[i]];
    for (int j = 0; j < c.size(); j++) out << litToInt(c[j]) << " ";
    out << "0\n";
  }
}  // displayToCnf

/**
 * @brief WrapperGlucose::uncheckedEnqueue implementation.
 */
void WrapperGlucose::uncheckedEnqueue(Lit l) {
  s.uncheckedEnqueue(Glucose::mkLit(l.var(), l.sign()));
}  // uncheckedEnqueue

/**
 * @brief WrapperGlucose::solve implementation.
 */
bool WrapperGlucose::solve() {
  s.rebuildWithAllVar();
  return s.solveWithAssumptions();
}  // solve

/**
 * @brief WrapperGlucose::addClauseInit implementation.
 */
void WrapperGlucose::addClauseInit(std::vector<Lit>& cl) {
  Glucose::vec<Glucose::Lit> addCl;
  for (auto& l : cl) addCl.push(Glucose::mkLit(l.var(), l.sign()));
  s.addClauseInit(addCl);
}  // addClauseInit

/**
 * @brief WrapperGlucose::setLastIndexAssumption implementation.
 */
void WrapperGlucose::setLastIndexAssumption(int lastIndex) {
  s.limitSelector = lastIndex;
}  // setLastIndexAssumption

/**
 * @brief WrapperGlucose::solveLimited implementation.
 */
Status WrapperGlucose::solveLimited(int nbConflict) {
  s.rebuildOrderHeap();
  Glucose::lbool res = s.solveLimited(nbConflict);
  if (res == Glucose::l_False) return UNS;
  if (res == Glucose::l_True) return SAT;
  return UND;
}  // solveLimited

/**
 * @brief WrapperGlucose::showTrail implementation.
 */
void WrapperGlucose::showTrail() { s.showTrail(); }  // showTrail

/**
 * @brief WrapperGlucose::setReversePolarity implementation.
 */
void WrapperGlucose::setReversePolarity(bool value) {
  s.reversePolarity = value;
}  // setReversePolarity

/**
 * @brief WrapperGlucose::decideAndComputeUnit implementation.
 */
bool WrapperGlucose::decideAndComputeUnit(Lit l, std::vector<Lit>& units) {
  Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());
  if (varIsAssigned(l.var())) {
    if (s.litAssigned(l.var()) != ml) return false;
    units.push_back(l);
    return true;
  }

  int posTrail = (s.trail).size();
  s.newDecisionLevel();
  s.uncheckedEnqueue(ml);
  Glucose::CRef confl = s.propagate();

  if (confl != Glucose::CRef_Undef)  // unit literal
  {
    int bt;
    Glucose::vec<Glucose::Lit> learnt_clause;
    s.analyzeLastUIP(confl, learnt_clause, bt);
    s.cancelUntil(s.decisionLevel() - 1);
    assert(learnt_clause[0] == Glucose::mkLit(l.var(), !l.sign()));
    s.insertClauseAndPropagate(learnt_clause);
    return false;
  }

  for (int j = posTrail; j < s.trail.size(); j++)
    units.push_back(Lit::makeLit(var(s.trail[j]), sign(s.trail[j])));
  s.cancelUntil(s.decisionLevel() - 1);
  return true;
}  // decideAndComputeUnit

/**
 * @brief WrapperGlucose::decideAndTest implementation.
 */
bool WrapperGlucose::decideAndTest(std::vector<Lit>& lits,
                                   std::vector<Lit>& core) {
  for (auto& l : lits) {
    Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());

    Glucose::CRef confl = Glucose::CRef_Undef;
    if (!s.isAssigned(l.var())) {
      s.newDecisionLevel();
      s.uncheckedEnqueue(ml);
      confl = s.propagate();

      if (confl != Glucose::CRef_Undef)  // conflict
      {
        Glucose::vec<Glucose::Lit> learnt_clause;
        s.analyzeFinal(confl, learnt_clause);

        for (int i = 0; i < learnt_clause.size(); i++) {
          core.push_back(
              Lit::makeLit(var(learnt_clause[i]), sign(learnt_clause[i])));
        }
        s.cancelUntil(0);
        return false;
      }
    } else if (s.litAssigned(l.var()) != ml)  // conflict!
    {
      Glucose::vec<Glucose::Lit> learnt_clause;
      s.analyzeFinal(~ml, learnt_clause);

      for (int i = 0; i < learnt_clause.size(); i++) {
        core.push_back(
            Lit::makeLit(var(learnt_clause[i]), sign(learnt_clause[i])));
      }
      s.cancelUntil(0);
      return false;
    }
  }

  s.cancelUntil(0);
  return true;
}  // decideAndTest

/**
 * @brief WrapperGlucose::getUnits implementation.
 */
void WrapperGlucose::getUnits(std::vector<Lit>& units) {
  for (int i = 0; i < s.trail.size(); i++) {
    Glucose::Lit l = s.trail[i];
    units.push_back(Lit::makeLit(var(l), sign(l)));
  }
}  // getUnits

/**
 * @brief WrapperGlucose::varIsAssigned implementation.
 */
bool WrapperGlucose::varIsAssigned(Var v) {
  return s.isAssigned(v);
}  // varIsAssigned

/**
 * @brief WrapperGlucose::restart implementation.
 */
void WrapperGlucose::restart() { s.cancelUntil(0); }  // restart

/**
 * @brief WrapperGlucose::setAssumption implementation.
 */
void WrapperGlucose::setAssumption(std::vector<Lit>& assums) {
  // get the place where the assumption are different.
  unsigned i = 0;
  for (; i < assums.size() && i < m_assumption.size(); i++)
    if (assums[i] != m_assumption[i]) break;

  // unset the literal as being in the assumption.
  for (unsigned j = i; j < m_assumption.size(); j++)
    m_isInAssumption[m_assumption[j].var()] = false;

  Glucose::vec<Glucose::Lit>& assumptions = s.assumptions;
  assert(assumptions.size() == m_assumption.size());
  assumptions.shrink_(assumptions.size() - i);
  m_assumption.resize(i);

  (s.cancelUntil)(assumptions.size());
  for (; i < assums.size(); i++) pushAssumption(assums[i]);
}  // setAssumption

/**
 * @brief WrapperGlucose::getAssumption implementation.
 */
std::vector<Lit>& WrapperGlucose::getAssumption() {
  return m_assumption;
}  // getAssumption

/**
 * @brief WrapperGlucose::displayAssumption implementation.
 */
void WrapperGlucose::displayAssumption(std::ostream& out) {
  Glucose::vec<Glucose::Lit>& assumptions = s.assumptions;
  for (int i = 0; i < assumptions.size(); i++) {
    Glucose::Lit l = assumptions[i];
    std::cout << (Glucose::sign(l) ? "-" : "") << Glucose::var(l) << " ";
  }
  std::cout << "\n";
}  // displayAssumption

/**
 * @brief WrapperGlucose::setNeedModel implementation.
 */
void WrapperGlucose::setNeedModel(bool b) {
  m_needModel = b;
  s.setNeedModel(b);
}  // setNeedModel

/**
 * @brief WrapperGlucose::getCore implementation.
 */
void WrapperGlucose::getCore(std::vector<Lit>& core) {
  core.clear();
  for (int i = 0; i < s.conflict.size(); i++)
    core.push_back(Lit::makeLit(var(s.conflict[i]), sign(s.conflict[i])));
}  // getCore

/**
 * @brief WrapperGlucose::getModel implementation.
 */
std::vector<bool>& WrapperGlucose::getModel() {
  assert(m_needModel);
  assert(s.model.size() <= m_model.size());

  for (int i = 0; i < s.model.size(); i++) {
    if (Glucose::toInt(s.model[i]) == 0)
      m_model[i] = true;
    else if (Glucose::toInt(s.model[i]) == 1)
      m_model[i] = false;
  }

  return m_model;
}  // getModel

/**
 * @brief WrapperGlucose::pushAssumption implementation.
 */
void WrapperGlucose::pushAssumption(Lit l) {
  Glucose::Lit ml = Glucose::mkLit(l.var(), l.sign());
  m_activeModel = m_activeModel && !s.isAssigned(var(ml));

  (s.assumptions).push(ml);
  m_assumption.push_back(l);
  assert((s.assumptions).size() == m_assumption.size());

  assert(!m_isInAssumption[l.var()]);
  m_isInAssumption[l.var()] = 1 + l.sign();

  if (m_activeModel && m_needModel) {
    m_activeModel = s.litTrueInLastModel(ml);
    if (m_activeModel) {
      assert(s.decisionLevel() == s.assumptions.size() - 1);
      s.newDecisionLevel();
      assert(!s.isAssigned(var(ml)));
      s.uncheckedEnqueue(ml);
      [[maybe_unused]] Glucose::CRef cref = s.propagate();
      assert(cref == Glucose::CRef_Undef);
    }
  }
}  // pushAssumption

/**
 * @brief WrapperGlucose::popAssumption implementation.
 */
void WrapperGlucose::popAssumption(unsigned count) {
  for (unsigned i = m_assumption.size() - count; i < m_assumption.size(); i++) {
    assert(m_isInAssumption[m_assumption[i].var()]);
    m_isInAssumption[m_assumption[i].var()] = 0;
  }
  m_assumption.resize(m_assumption.size() - count);
  (s.assumptions).shrink_(count);
  (s.cancelUntil)((s.assumptions).size());
}  // popAssumption

/**
 * @brief WrapperGlucose::oncePriorityVar implementation.
 */
void WrapperGlucose::oncePriorityVar(std::vector<Var>& priority) {
  s.priorityVar.clear();
  for (auto v : priority) s.priorityVar.push(v);
}

/**
 * @brief WrapperGlucose::setHeapVariable implementation.
 */
void WrapperGlucose::setHeapVariable(std::vector<Var>& heapVar) {
  s.problemVariable.clear();
  for (auto v : heapVar) s.problemVariable.push(v);
}  // oncePriorityVar

/**
 * @brief WrapperGlucose::cleanAssumption implementation.
 */
void WrapperGlucose::cleanAssumption() {
  for (auto& l : m_assumption) m_isInAssumption[l.var()] = false;

  m_assumption.clear();
  s.assumptions.clear();
  s.cancelUntil(0);
}  // cleanAssumption

}  // namespace bipe