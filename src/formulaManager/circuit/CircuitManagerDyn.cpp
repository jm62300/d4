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

#include "CircuitManagerDyn.hpp"

namespace d4 {

/**
 * @brief CircuitManagerDyn::CircuitManagerDyn implementation.
 */
CircuitManagerDyn::CircuitManagerDyn(const ProblemManager& p, bool optRmGates)
    : FormulaManager(p.getNbVar()),
      CircuitManager(p, optRmGates) {
  unsigned nbVar = getNbVariable();
  unsigned nbGates = m_gates.size();

  // Build per-variable gate occurrence lists.
  // CLAUSE gates (T-statements): add all input variables.
  // Non-CLAUSE gates: add the output variable and all input variables.
  m_varGates.resize(nbVar + 1);
  for (unsigned i = 0; i < nbGates; i++) {
    if (m_gates[i].gateType == BcGateType::CLAUSE) {
      for (auto l : m_gates[i].input) m_varGates[l.var()].push_back(i);
    } else {
      m_varGates[m_gates[i].output.var()].push_back(i);
      for (auto l : m_gates[i].input) m_varGates[l.var()].push_back(i);
    }
  }

  // Initialize alive gate counts.  The parent constructor may have already
  // deactivated some gates (when optRmGates is true), so we count only alive
  // gate entries.
  m_nbActiveGates.assign(nbVar + 1, 0);
  for (unsigned v = 1; v <= nbVar; v++)
    for (unsigned idx : m_varGates[v])
      if (isAliveGateIdx(idx)) m_nbActiveGates[v]++;

  // Allocate union-find scratch structures.
  m_activeVarsCirc = new Var[nbVar + 1];
  m_infoClusterCirc.resize(nbVar + nbGates + 1, {0, 0, -1});
  m_markCirc.assign(nbGates, 0);
  m_stampCirc = 1;
  m_idxComponentCirc.assign(nbVar + 1, 0);
}

/**
 * @brief CircuitManagerDyn::~CircuitManagerDyn implementation.
 */
CircuitManagerDyn::~CircuitManagerDyn() { delete[] m_activeVarsCirc; }

/**
 * @brief Returns true if gate idx still contributes to connectivity.
 *
 * Mirrors Tseitin clause-satisfaction semantics so the gate-based CC matches
 * the CNF-based one exactly:
 *
 *   CLAUSE  {l1…ln}:  alive iff no li is assigned to true (clause unsatisfied)
 *   AND     o=i1∧…:   alive iff o is NOT assigned true
 *                     (o=false → residual {~i1,…,~in} still active;
 *                      o=true  → backward units force all inputs true →
 * assigned) OR      o=i1∨…:   alive iff o is NOT assigned false (o=true  →
 * residual {i1,…,in} still active; o=false → backward units force all inputs
 * false → assigned) IDENTITY o↔i1:    alive iff o is unassigned (assignment
 * forces the other)
 */
inline bool CircuitManagerDyn::isAliveGateIdx(unsigned idx) { return 0; }

/**
 * @brief Wrap-around stamp increment (mirrors
 * CnfManager::incrementStampMarkView).
 */
inline void CircuitManagerDyn::incrementStampCirc() {
  if (m_stampCirc == std::numeric_limits<uint32_t>::max()) {
    std::fill(m_markCirc.begin(), m_markCirc.end(), 0);
    m_stampCirc = 1;
  } else {
    m_stampCirc++;
  }
}

/**
 * @brief isFreeVariable implementation: a variable is free when no currently
 * alive gate (per isAliveGateIdx) references it.
 *
 * Uses isAliveGateIdx rather than m_nbActiveGates so that assignment-induced
 * gate resolution (e.g. OR gate with true output, AND gate with false output)
 * is accounted for — matching the CNF clause-satisfaction semantics.
 */
bool CircuitManagerDyn::isFreeVariable(Var v) {
  for (unsigned gIdx : m_varGates[v])
    if (isAliveGateIdx(gIdx)) return false;
  return true;
}

/**
 * @brief computeConnectedComponent implementation using union-find over gates.
 *
 * Direct mirror of CnfManager::computeConnectedComponent (cnf/CnfManager.cpp
 * lines 129–219), substituting gate indices for clause indices.
 */
int CircuitManagerDyn::computeConnectedComponent(
    std::vector<std::vector<Var>>& varConnected, std::span<Var> setOfVar,
    std::vector<Var>& freeVar) {
  return 0;
}

/**
 * @brief computeConnectedComponentTargeted implementation using BFS over gates.
 *
 * Mirror of CnfManager::computeConnectedComponentTargeted
 * (cnf/CnfManager.cpp lines 268–319), substituting gate traversal for clause
 * traversal via connectedToLit.
 */
int CircuitManagerDyn::computeConnectedComponentTargeted(
    std::vector<std::vector<Var>>& varConnected, std::vector<Var>& setOfVar,
    std::vector<bool>& isTargeted, std::vector<Var>& freeVar) {
  return 0;
}

void CircuitManagerDyn::preUpdate(const std::vector<Lit>& lits) { exit(1); }
void CircuitManagerDyn::postUpdate(const std::vector<Lit>& lits) { exit(1); }

}  // namespace d4
