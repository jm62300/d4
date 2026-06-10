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

#include "CircuitManager.hpp"

#include "src/problem/ProblemManager.hpp"

#define TEST_DEBUG 0

namespace d4 {

/**
 * @brief CircuitManager::CircuitManager implementation.
 */
CircuitManager::CircuitManager(const ProblemManager& p, bool optRmGates)
    : FormulaManager(p.getNbVar()) {
  std::cout << "c [CIRCUIT MANAGER] Constructor called\n";

  m_gates = p.getGates();
  m_optionRemoveGates = optRmGates;
  m_propagatedFree = 0;

  std::vector<BcGate> tseitinGates;
  p.toCnf(tseitinGates);
  ProblemManager tseitinProblem("cnf", p.getNbVar(), p.getQuantification(),
                                p.getWeightMap(), tseitinGates, std::cout);
  m_cnfManager = new CnfManagerDyn(tseitinProblem);

  // Populate m_true_lits from CLAUSE gates (T statements).
  for (const auto& g : m_gates)
    if (g.gateType == BcGateType::CLAUSE)
      for (const auto& l : g.input) m_true_lits.push_back(l);

  m_lastIndex = m_gates.size();
  m_varToGate.resize(p.getNbVar() + 1, m_lastIndex);
  m_gateToVar.resize(m_gates.size());
  m_varInputInGates.resize(p.getNbVar() + 1);
  m_isStillAlive.resize(p.getNbVar() + 1, true);
  m_litThatInactiveVar.resize((p.getNbVar() + 1) << 1);

  for (unsigned i = 0; i < m_gates.size(); i++) {
    if (m_gates[i].gateType == BcGateType::CLAUSE) continue;

    for (auto l : m_gates[i].input) {
      if (m_gates[i].gateType == BcGateType::AND) l = ~l;
      m_litThatInactiveVar[l.intern()].push_back(m_gates[i].output.var());
    }

    assert(m_varToGate[m_gates[i].output.var()] == m_lastIndex);
    m_varToGate[m_gates[i].output.var()] = i;
    m_gateToVar[i] = m_gates[i].output.var();

    for (auto& l : m_gates[i].input)
      m_varInputInGates[l.var()].push_back(m_gates[i].output.var());
  }

  std::vector<bool> isUnit(p.getNbVar() + 1, false);
  for (auto& l : m_true_lits) isUnit[l.var()] = true;

  if (m_optionRemoveGates) {
    std::vector<Var> shouldBePropagated;

    bool modif = true;
    while (modif) {
      modif = false;

      for (unsigned i = 1; i <= p.getNbVar(); i++) {
        if (!m_isStillAlive[i] || m_varToGate[i] == m_lastIndex || isUnit[i] ||
            m_gates[m_varToGate[i]].gateType == BcGateType::IDENTITY)
          continue;

        if (!m_varInputInGates[i].size()) {
          modif = true;
          shouldBePropagated.push_back(i);
          m_isStillAlive[i] = false;

          for (auto& l : m_gates[m_varToGate[i]].input) {
            for (unsigned j = 0; j < m_varInputInGates[l.var()].size(); j++) {
              if (m_varInputInGates[l.var()][j] == i) {
                m_varInputInGates[l.var()][j] =
                    m_varInputInGates[l.var()].back();
                m_varInputInGates[l.var()].pop_back();
                break;
              }
            }
          }
        }
      }
    }

    m_watchList.resize(p.getNbVar() + 1);
    for (unsigned i = 1; i <= p.getNbVar(); i++) {
      if (m_varToGate[i] == m_lastIndex ||
          m_gates[m_varToGate[i]].gateType == BcGateType::IDENTITY)
        continue;

      if (m_varInputInGates[i].size())
        m_watchList[m_varInputInGates[i][0]].push_back(i);
    }

    if (shouldBePropagated.size()) {
      std::vector<Lit> litsTrue;
      for (auto& v : shouldBePropagated) {
        litsTrue.push_back(Lit::makeLitTrue(v));
        litsTrue.push_back(Lit::makeLitFalse(v));
      }

      m_stackGatesNotAliveSize.push_back(m_stackGatesNotAlive.size());
      m_cnfManager->pushStacks();
      m_cnfManager->propagateTrue(litsTrue);

      m_propagatedFree += shouldBePropagated.size();
      m_cnfManager->unmarkLastClausesSaved();
    }
  }
}  // constructor

/**
 * @brief CircuitManager::~CircuitManager implementation.
 */
CircuitManager::~CircuitManager() {
  std::cout << "c [CIRCUIT MANAGER] Destructor called\n";
  delete m_cnfManager;
}  // destructor

/**
 * @brief CircuitManager::stillActive implementation.
 */
bool CircuitManager::stillActive(BcGate& g) {
  assert(g.output.var() <= getNbVariable());

  if (!varIsAssigned(g.output.var())) {
    return true;
  }
  Lit& l = g.output;
  switch (g.gateType) {
    case BcGateType::OR:
      if (litIsAssignedToTrue(~l)) return false;
      for (auto& m : g.input)
        if (litIsAssignedToTrue(m)) return false;
      return true;
    case BcGateType::AND:
      if (litIsAssignedToTrue(l)) return false;
      for (auto& m : g.input)
        if (litIsAssignedToTrue(~m)) return false;
      return true;
    default:
      return true;
  }
}  // stillActive

/**
 * @brief CircuitManager::propagate implementation.
 */
void CircuitManager::propagate(std::vector<Var>& vars,
                               std::vector<Var>& pVars) {
  while (vars.size()) {
    Var v = vars.back();
    assert(!m_isStillAlive[v]);
    vars.pop_back();

    unsigned i, j;
    for (i = j = 0; i < m_watchList[v].size(); i++) {
      Var w = m_watchList[v][i];

      if (!m_isStillAlive[w] || varIsAssigned(w))
        m_watchList[v][j++] = w;
      else {
        int next = var_Undef;
        for (auto& x : m_varInputInGates[w]) {
          if (m_isStillAlive[x]) {
            next = x;
            break;
          }
        }

        assert(next != v);
        if (next != var_Undef)
          m_watchList[next].push_back(w);
        else {
          m_watchList[v][j++] = w;
          vars.push_back(w);

          pVars.push_back(w);
          m_isStillAlive[w] = false;
          m_stackGatesNotAlive.push_back(w);
          onGateDeactivated(w);
        }
      }
    }
    m_watchList[v].resize(j);
  }
}  // propagate

/**
 * @brief CircuitManager::computeTrivialConnectedComponent implementation.
 */
int CircuitManager::computeTrivialConnectedComponent(
    std::vector<std::vector<Var>>& varConnected, std::span<Var> setOfVar,
    std::vector<Var>& freeVar) {
  varConnected.push_back(std::vector<Var>());
  for (auto& v : setOfVar) {
    if (varIsAssigned(v) || !m_isStillAlive[v]) continue;
    if (isFreeVariable(v))
      freeVar.push_back(v);
    else
      varConnected[0].push_back(v);
  }
  if (!varConnected[0].size()) varConnected.pop_back();

  return varConnected.size();
}  // computeTrivialConnectedComponent

/**
 * @brief CircuitManager::debugFunction implementation.
 */
void CircuitManager::debugFunction() {
  for (unsigned i = 1; i <= getNbVariable(); i++) {
    if (varIsAssigned(i) || m_varToGate[i] == m_lastIndex) continue;

    bool shouldBeDead = true;
    for (auto& v : m_varInputInGates[i]) {
      shouldBeDead = !m_isStillAlive[v];
      if (!shouldBeDead) break;
    }

    if (shouldBeDead == m_isStillAlive[i])
      std::cout << "+++ " << i << " " << shouldBeDead << '\n';
    assert(shouldBeDead != m_isStillAlive[i]);
  }
}  // debugFunction

/**
 * @brief CircuitManager::preUpdate implementation.
 */
void CircuitManager::preUpdateGatesRemoved(const std::vector<Lit>& lits,
                                           std::vector<Lit>& litsTrue) {
  litsTrue = lits;
  if (!m_optionRemoveGates) return;

  m_stackGatesNotAliveSize.push_back(m_stackGatesNotAlive.size());

  std::vector<Var> toPu, puVars;
  for (auto& l : lits) {
    for (auto& v : m_litThatInactiveVar[l.intern()]) {
      if (m_isStillAlive[v]) {
        m_isStillAlive[v] = false;
        m_stackGatesNotAlive.push_back(v);
        onGateDeactivated(v);
        toPu.push_back(v);
      }
    }

    if (m_varToGate[l.var()] == m_lastIndex) continue;
    assert(m_varToGate[l.var()] < m_gates.size());
    m_stackGatesNotAlive.push_back(l.var());
  }

  propagate(toPu, puVars);
  m_propagatedFree += puVars.size();

#define JUMP 1000000
  static unsigned limit = JUMP;
  if (m_propagatedFree > limit) {
    std::cout << "#gates removed: " << m_propagatedFree << '\n';
    limit += JUMP;
  }

  for (auto& v : puVars) {
    litsTrue.push_back(Lit::makeLitTrue(v));
    litsTrue.push_back(Lit::makeLitFalse(v));
  }

}  // preUpdateGatesRemoved

/**
 * @brief CircuitManager::postUpdate implementation.
 */
void CircuitManager::postUpdateGatesRemoved(const std::vector<Lit>& lits) {
  if (!m_optionRemoveGates) return;

  assert(m_stackGatesNotAliveSize.size());
  for (unsigned i = m_stackGatesNotAliveSize.back();
       i < m_stackGatesNotAlive.size(); i++)
    m_isStillAlive[m_stackGatesNotAlive[i]] = true;
  m_stackGatesNotAlive.resize(m_stackGatesNotAliveSize.back());
  m_stackGatesNotAliveSize.pop_back();
}  // postUpdateGatesRemoved

/**
 * @brief CircuitManager::showFormula implementation.
 */
void CircuitManager::showFormula(std::ostream& out) {
  m_cnfManager->showFormula(out);
}

/**
 * @brief CircuitManager::showCurrentFormula implementation.
 */
void CircuitManager::showCurrentFormula(std::ostream& out) {
  m_cnfManager->showCurrentFormula(out);
}

/**
 * @brief CircuitManager::showCurrentFormula (with component mask)
 * implementation.
 */
void CircuitManager::showCurrentFormula(std::ostream& out,
                                        std::vector<bool>& isInComponent) {
  m_cnfManager->showCurrentFormula(out, isInComponent);
}

/**
 * @brief CircuitManager::printInformation implementation.
 */
void CircuitManager::printInformation(std::ostream& out) {
  out << "c \033[1m\033[36mFormula Manager Information\033[0m\n";
  m_cnfManager->printInformation(out);
  out << "c Number of variable eliminated: " << m_propagatedFree << '\n';
  out << "c\n";
}

/**
 * @brief CircuitManager::isFreeVariable implementation.
 * Default: delegates to the embedded CNF manager.
 */
bool CircuitManager::isFreeVariable(Var v) {
  return m_cnfManager->isFreeVariable(v);
}

/**
 * @brief CircuitManager::initFormulaStore implementation.
 */
void CircuitManager::initFormulaStore(const OptionBucketManager& opts) {
  m_cnfManager->initFormulaStore(opts);
}

/**
 * @brief CircuitManager::storeFormula implementation.
 */
void CircuitManager::storeFormula(std::span<const Var> component, DataBucket& b,
                                  BucketAllocator& alloc) {
  m_cnfManager->storeFormula(component, b, alloc);
}

}  // namespace d4
