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

#include "CircuitWithCnfManager.hpp"

#include "src/problem/ProblemManager.hpp"

namespace d4 {

/**
 * @brief CircuitWithCnfManager::CircuitWithCnfManager implementation.
 */
CircuitWithCnfManager::CircuitWithCnfManager(ProblemManager &p, bool optRmGates)
    : CircuitManager(p) {
  std::cout << "c [CIRCUIT WITH CNF MANAGER] Constructor called\n";

  try {
    ProblemManagerCircuit &pcirc = dynamic_cast<ProblemManagerCircuit &>(p);
    m_problemCnf = static_cast<ProblemManagerCnf *>(
        pcirc.translate(ProblemTranslateType::TRANSLATE_CNF));
  } catch (std::bad_cast &bc) {
    std::cerr << "c bad_cast caught: " << bc.what() << '\n';
    std::cerr << "c A boolean circuit was expected\n";
    assert(0);
  }

  m_optionRemoveGates = optRmGates;
  m_propagatedFree = 0;
  m_cnfManager = new CnfManagerDyn(*m_problemCnf);

  // assign variable with their definition, and for each variable v store the
  // gates where v is part of the input.
  m_lastIndex = m_gates.size();
  m_varToGate.resize(p.getNbVar() + 1, m_lastIndex);
  m_gateToVar.resize(m_gates.size());
  m_varInputInGates.resize(p.getNbVar() + 1);
  m_isStillAlive.resize(p.getNbVar() + 1, true);
  m_litThatInactiveVar.resize((p.getNbVar() + 1) << 1);

  for (unsigned i = 0; i < m_gates.size(); i++) {
    m_gates[i].display(std::cout);
    std::cout << '\n';

    for (auto l : m_gates[i].input) {
      if (m_gates[i].gate_type == BcGateType::AND) l = ~l;
      m_litThatInactiveVar[l.intern()].push_back(m_gates[i].output.var());
    }

    assert(m_varToGate[m_gates[i].output.var()] == m_lastIndex);
    m_varToGate[m_gates[i].output.var()] = i;
    m_gateToVar[i] = m_gates[i].output.var();

    for (auto &l : m_gates[i].input)
      m_varInputInGates[l.var()].push_back(m_gates[i].output.var());
  }

  std::vector<bool> isUnit(p.getNbVar() + 1, false);
  for (auto &l : m_true_lits) isUnit[l.var()] = true;

  // set the watch list.
  std::vector<Var> shouldBePropagated;
  m_watchList.resize(p.getNbVar() + 1);
  for (unsigned i = 1; i <= p.getNbVar(); i++) {
    if (m_varToGate[i] == m_lastIndex ||
        m_gates[m_varToGate[i]].gate_type == BcGateType::IDENTITY)
      continue;  // input var.

    if (m_varInputInGates[i].size())
      m_watchList[m_varInputInGates[i][0]].push_back(i);
    else if (!isUnit[i]) {
      m_isStillAlive[i] = false;
      m_stackGatesNotAlive.push_back(i);
      shouldBePropagated.push_back(i);
      std::cout << "should propage " << i << '\n';
    }
  }

  std::vector<Var> puVars;
  propagate(shouldBePropagated, puVars);
  m_propagatedFree += puVars.size();
}  // constructor

/**
 * @brief CircuiWithCnfManager::CircuitWithCnfManager implementation.
 */
CircuitWithCnfManager::~CircuitWithCnfManager() {
  std::cout << "c [CIRCUIT WITH CNF MANAGER] Destructor called\n";

}  // constructor

/**
 * @brief CircuitWithCnfManager::stillActive implementation.
 */
bool CircuitWithCnfManager::stillActive(BcGate &g) {
  assert(g.output.var() <= getNbVariable());

  if (!varIsAssigned(g.output.var())) {
    return true;
  }
  Lit &l = g.output;
  switch (g.gate_type) {
    case BcGateType::OR:
      if (litIsAssignedToTrue(~l)) return false;
      for (auto &m : g.input)
        if (litIsAssignedToTrue(m)) return false;
      return true;
    case BcGateType::AND:
      if (litIsAssignedToTrue(l)) return false;
      for (auto &m : g.input)
        if (litIsAssignedToTrue(~m)) return false;
      return true;
    default:
      return true;
  }
}  // stillActive

/**
 * @brief CircuitWithCnfManager::propagate implementation.
 */
void CircuitWithCnfManager::propagate(std::vector<Var> &vars,
                                      std::vector<Var> &pVars) {
  while (vars.size()) {
    Var v = vars.back();
    vars.pop_back();

    std::cout << "v = " << v << '\n';

    unsigned i, j;
    for (i = j = 0; i < m_watchList[v].size(); i++) {
      Var w = m_watchList[v][i];
      std::cout << v << " ~~> " << w << " "
                << stillActive(m_gates[m_varToGate[w]]) << '\n';

      if (!m_isStillAlive[w] || varIsAssigned(w))
        m_watchList[v][j++] = w;
      else {
        std::cout << "look for another watch\n";

        // search another watch for w.
        int next = var_Undef;
        for (auto &x : m_varInputInGates[w]) {
          if (!m_cnfManager->varIsAssigned(x) || m_isStillAlive[x]) {
            next = x;
            std::cout << "find a watch: " << x << '\n';
            break;
          }
        }

        if (next != var_Undef)
          m_watchList[next].push_back(w);
        else {
          m_watchList[v][j++] = w;
          vars.push_back(w);

          pVars.push_back(w);
          m_isStillAlive[w] = false;
          m_stackGatesNotAlive.push_back(w);
        }
      }
    }
    m_watchList[v].resize(j);
  }
}  // propagate

/**
 * @brief CircuitWithCnfManager::computeConnectedComponent implementation.
 */
int CircuitWithCnfManager::computeConnectedComponent(
    std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
    std::vector<Var> &freeVar) {
  return m_cnfManager->computeConnectedComponent(varConnected, setOfVar,
                                                 freeVar);
}  // computeConnectedComponent

/**
 * @brief CircuitWithCnfManager::computeConnectedComponentTargeted
 * implementation.
 */
int CircuitWithCnfManager::computeConnectedComponentTargeted(
    std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
    std::vector<bool> &isTargeted, std::vector<Var> &freeVar) {
  // compute the connected component on the cnfManager
  int ret = m_cnfManager->computeConnectedComponentTargeted(
      varConnected, setOfVar, isTargeted, freeVar);

  // remove the free variables that are no more activated.
  unsigned i, j;
  for (i = j = 0; i < freeVar.size(); i++)
    if (m_isStillAlive[freeVar[i]]) freeVar[j++] = freeVar[i];
  freeVar.resize(j);

  // return the number of connected component.
  return ret;
}  // computeConnectedComponentTargeted

/**
 * @brief CircuitWithCnfManager::debugFunction implementation.
 */
void CircuitWithCnfManager::debugFunction() {
  for (unsigned i = 1; i <= getNbVariable(); i++) {
    if (varIsAssigned(i) || m_varToGate[i] == m_lastIndex) continue;

    bool shouldBeDead = true;
    for (auto &v : m_varInputInGates[i]) {
      shouldBeDead = !m_isStillAlive[v];
      if (!shouldBeDead) break;
    }

    if (shouldBeDead == m_isStillAlive[i]) std::cout << "+++ " << i << '\n';

    assert(shouldBeDead != m_isStillAlive[i]);
  }
}  // debugFunction

/**
 * @brief CircuitWithCnfManager::preUpdate implementation.
 */
void CircuitWithCnfManager::preUpdate(const std::vector<Lit> &lits) {
  m_stackGatesNotAliveSize.push_back(m_stackGatesNotAlive.size());

  m_cnfManager->pushStacks();
  assignListLit(lits);
  m_cnfManager->assignListLit(lits);

  std::vector<Lit> litsTrue = lits;
  if (m_optionRemoveGates) {
    std::vector<Var> toPu, puVars;
    std::cout << "lit assigned: ";
    for (auto &l : lits) std::cout << l << ' ';
    std::cout << '\n';

    for (auto &l : lits) {
      for (auto &v : m_litThatInactiveVar[l.intern()])
        if (m_isStillAlive[v]) {
          m_isStillAlive[v] = false;
          m_stackGatesNotAlive.push_back(v);
          toPu.push_back(v);
        }

      std::cout << "=======> " << l << '\n';
      if (m_varToGate[l.var()] == m_lastIndex) continue;
      assert(m_varToGate[l.var()] < m_gates.size());
      m_isStillAlive[l.var()] = stillActive(m_gates[m_varToGate[l.var()]]);
      m_stackGatesNotAlive.push_back(l.var());
    }

    propagate(toPu, puVars);
    m_propagatedFree += puVars.size();

    debugFunction();
    for (auto &v : puVars) {
      litsTrue.push_back(Lit::makeLitTrue(v));
      litsTrue.push_back(Lit::makeLitFalse(v));
    }
  }

  m_cnfManager->propagateTrue(litsTrue);
  m_cnfManager->propagateFalseInNotBin(lits);

  // unmark the clauses.
  m_cnfManager->unmarkLastClausesSaved();
}  // preUpdate

/**
 * @brief CircuitWithCnfManager::postUpdate implementation.
 */
void CircuitWithCnfManager::postUpdate(const std::vector<Lit> &lits) {
  for (auto &l : lits) m_currentValue[l.var()] = l_Undef;

  // backtrack.
  assert(m_stackGatesNotAliveSize.size());
  for (unsigned i = m_stackGatesNotAliveSize.back();
       i < m_stackGatesNotAlive.size(); i++)
    m_isStillAlive[m_stackGatesNotAlive[i]] = true;
  m_stackGatesNotAlive.resize(m_stackGatesNotAliveSize.back());
  m_stackGatesNotAliveSize.pop_back();

  m_cnfManager->postUpdate(lits);
}  // postUpdate

/**
 * @brief CircuitWithCnfManager::showFormula implementation.
 */
void CircuitWithCnfManager::showFormula(std::ostream &out) {
  m_cnfManager->showFormula(out);
}  // showFormula

/**
 * @brief CircuitWithCnfManager::showCurrentFormula implementation.
 */
void CircuitWithCnfManager::showCurrentFormula(std::ostream &out) {
  m_cnfManager->showCurrentFormula(out);
}  // showCurrentFormula

/**
 * @brief CircuitWithCnfManager::showCurrentFormula implementation.
 */
void CircuitWithCnfManager::showCurrentFormula(
    std::ostream &out, std::vector<bool> &isInComponent) {
  m_cnfManager->showCurrentFormula(out, isInComponent);
}  // showCurrentFormula

/**
 * @brief CircuitWithCnfManager::getProblemInputType implementation.
 */
ProblemInputType CircuitWithCnfManager::getProblemInputType() {
  return PB_CIRC;
}  // getProblemType

/**
 * @brief CircuitWithCnfManager::printInformation implementation.
 */
void CircuitWithCnfManager::printInformation(std::ostream &out) {
  out << "c \033[1m\033[36mFormula Manager Information\033[0m\n";
  m_cnfManager->printInformation(out);
  out << "c Number of variable eliminated: " << m_propagatedFree << '\n';
  out << "c\n";
}  // printInformation

/**
 * CircuitWithCnfManager::isFreeVariable implementation.
 */
bool CircuitWithCnfManager::isFreeVariable(Var v) {
  return m_cnfManager->isFreeVariable(v);
}  // isFreeVariable

}  // namespace d4
