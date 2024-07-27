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
CircuitWithCnfManager::CircuitWithCnfManager(ProblemManager &p)
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

  m_propagatedFree = 0;
  m_cnfManager = new CnfManagerDyn(*m_problemCnf);

  // assign variable with their definition, and for each variable v store the
  // gates where v is part of the input.
  m_lastIndex = m_gates.size();
  m_varToGate.resize(p.getNbVar() + 1, m_lastIndex);
  m_gateToVar.resize(m_gates.size());
  m_varInputInGates.resize(p.getNbVar() + 1);

  for (unsigned i = 0; i < m_gates.size(); i++) {
    if (m_gates[i].gate_type == BcGateType::IDENTITY) continue;

    assert(m_varToGate[m_gates[i].output.var()] == m_lastIndex);
    m_varToGate[m_gates[i].output.var()] = i;
    m_gateToVar[i] = m_gates[i].output.var();

    for (auto &l : m_gates[i].input)
      m_varInputInGates[l.var()].push_back(m_gates[i].output.var());
  }

  // set the watch list.
  std::vector<Var> shouldBePropagated;
  m_watchList.resize(p.getNbVar() + 1);
  for (unsigned i = 1; i <= p.getNbVar() + 1; i++) {
    if (m_varToGate[i] == m_lastIndex) continue;  // input var.

    if (m_varInputInGates[i].size())
      m_watchList[m_varInputInGates[i][0]].push_back(i);
    else
      shouldBePropagated.push_back(i);
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
 * @brief CircuitWithCnfManager::propagate implementation.
 */
void CircuitWithCnfManager::propagate(std::vector<Var> &vars,
                                      std::vector<Var> pVars) {
  while (vars.size()) {
    Var v = vars.back();
    vars.pop_back();
    pVars.push_back(v);

    unsigned i, j;
    for (i = j = 0; i < m_watchList[v].size(); i++) {
      Var w = m_watchList[v][i];
      if (m_currentValue[w] != l_Undef)
        m_watchList[v][j++] = w;
      else {
        // search another watch for w.
        int next = var_Undef;
        for (auto &x : m_varInputInGates[w]) {
          if (m_currentValue[x] == l_Undef) {
            next = x;
            break;
          }

          // only work when AND/OR gates are considered!
          BcGate &g = m_gates[m_varToGate[x]];
          switch (g.gate_type) {
            case BcGateType::AND:
              /* code */
              break;

            case BcGateType::OR:
              break;
            default:
              break;
          }
        }

        if (next != var_Undef)
          m_watchList[next].push_back(w);
        else {
          m_watchList[v][j++] = w;
          vars.push_back(w);
        }
      }
    }
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
  return m_cnfManager->computeConnectedComponentTargeted(varConnected, setOfVar,
                                                         isTargeted, freeVar);
}  // computeConnectedComponentTargeted

/**
 * @brief CircuitWithCnfManager::preUpdate implementation.
 */
void CircuitWithCnfManager::preUpdate(const std::vector<Lit> &lits) {
  m_cnfManager->pushStacks();
  m_cnfManager->assignListLit(lits);

  std::vector<Var> toPu, puVars;
  toPu.reserve(lits.size());
  for (auto &l : lits) toPu.push_back(l.var());
  propagate(toPu, puVars);

  // manage the non binary clauses.
  m_cnfManager->propagateTrue(lits);
  m_cnfManager->propagateFalseInNotBin(lits);

  // unmark the clauses.
  m_cnfManager->unmarkLastClausesSaved();
}  // preUpdate

/**
 * @brief CircuitWithCnfManager::postUpdate implementation.
 */
void CircuitWithCnfManager::postUpdate(const std::vector<Lit> &lits) {
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
  m_cnfManager->printInformation(out);
}  // printInformation

/**
 * CircuitWithCnfManager::isFreeVariable implementation.
 */
bool CircuitWithCnfManager::isFreeVariable(Var v) {
  return m_cnfManager->isFreeVariable(v);
}  // isFreeVariable

}  // namespace d4
