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
    : CircuitManager(p, optRmGates) {
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
inline bool CircuitManagerDyn::isAliveGateIdx(unsigned idx) {
  const BcGate& g = m_gates[idx];

  // CLAUSE {l1…ln}: alive iff the clause is unsatisfied (no li true).
  if (g.gateType == BcGateType::CLAUSE) {
    for (auto l : g.input)
      if (litIsAssignedToTrue(l)) return false;
    return true;
  }

  // Structural deactivation (optRmGates=true path).
  if (!m_isStillAlive[g.output.var()]) return false;

  // Output unassigned: gate fully active.
  if (!varIsAssigned(g.output.var())) return true;

  switch (g.gateType) {
    case BcGateType::AND:
      // o=TRUE  → backward units assign all inputs TRUE → no residual.
      if (litIsAssignedToTrue(g.output)) return false;
      // o=FALSE → Tseitin residual {~l1,…,~ln}.
      // Residual is satisfied iff any li is assigned FALSE, i.e. ~li is true.
      for (auto l : g.input)
        if (litIsAssignedToTrue(~l)) return false;
      return true;

    case BcGateType::OR:
      // o=FALSE → backward units assign all inputs FALSE → no residual.
      if (litIsAssignedToTrue(~g.output)) return false;
      // o=TRUE  → Tseitin residual {l1,…,ln}.
      // Residual is satisfied iff any li is assigned TRUE.
      for (auto l : g.input)
        if (litIsAssignedToTrue(l)) return false;
      return true;

    case BcGateType::IDENTITY:
      // o assigned → the unique input is also assigned by BCP.
      return false;

    default:
      return false;
  }
}

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
 * @brief Hook called by CircuitManager::propagate() and preUpdate() when a gate
 * with output variable w transitions from alive to inactive.
 */
void CircuitManagerDyn::onGateDeactivated(Var w) {
  unsigned gIdx = m_varToGate[w];
  assert(gIdx != m_lastIndex);
  m_nbActiveGates[w]--;
  for (auto l : m_gates[gIdx].input) m_nbActiveGates[l.var()]--;
}

/**
 * @brief Hook called by CircuitManager::postUpdate() when a gate with output
 * variable w is restored to alive during backtracking.
 */
void CircuitManagerDyn::onGateReactivated(Var w) {
  unsigned gIdx = m_varToGate[w];
  assert(gIdx != m_lastIndex);
  m_nbActiveGates[w]++;
  for (auto l : m_gates[gIdx].input) m_nbActiveGates[l.var()]++;
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
  incrementStampCirc();

  unsigned nbVar = getNbVariable();
  Var* lastVar = m_activeVarsCirc;
  Var* currentVar = m_activeVarsCirc;

  for (auto v : setOfVar) {
    assert(v <= nbVar);
    m_infoClusterCirc[v].parent = v;
    m_infoClusterCirc[v].size = 1;
    if (varIsAssigned(v) || !m_isStillAlive[v]) continue;
    if (!isFreeVariable(v))
      *lastVar++ = v;
    else
      freeVar.push_back(v);
  }

  while (currentVar != lastVar) {
    Var v = *currentVar++;
    Var rootV = v;

    for (unsigned gIdx : m_varGates[v]) {
      if (!isAliveGateIdx(gIdx)) continue;

      unsigned nodeIdx = gIdx + nbVar + 1;
      if (m_markCirc[gIdx] != m_stampCirc) {
        // First time we see this gate: register it under rootV.
        m_markCirc[gIdx] = m_stampCirc;
        m_infoClusterCirc[nodeIdx].parent = rootV;
        m_infoClusterCirc[rootV].size++;
      } else {
        // Gate already visited: find its current root and union with rootV.
        Var rootW = m_infoClusterCirc[nodeIdx].parent;
        while (rootW != m_infoClusterCirc[rootW].parent) {
          m_infoClusterCirc[rootW].parent =
              m_infoClusterCirc[m_infoClusterCirc[rootW].parent].parent;
          rootW = m_infoClusterCirc[rootW].parent;
        }
        if (rootV == rootW) continue;

        // Union by size.
        if (m_infoClusterCirc[rootV].size < m_infoClusterCirc[rootW].size) {
          m_infoClusterCirc[rootW].size += m_infoClusterCirc[rootV].size;
          m_infoClusterCirc[rootV].parent = m_infoClusterCirc[rootW].parent;
          rootV = rootW;
        } else {
          m_infoClusterCirc[rootV].size += m_infoClusterCirc[rootW].size;
          m_infoClusterCirc[rootW].parent = m_infoClusterCirc[rootV].parent;
        }
      }
    }
  }

  // Collect components.
  m_rootSetCirc.clear();
  for (currentVar = m_activeVarsCirc; currentVar != lastVar; currentVar++) {
    Var v = *currentVar;
    Var rootV = m_infoClusterCirc[v].parent;
    while (rootV != m_infoClusterCirc[rootV].parent) {
      m_infoClusterCirc[rootV].parent =
          m_infoClusterCirc[m_infoClusterCirc[rootV].parent].parent;
      rootV = m_infoClusterCirc[rootV].parent;
    }

    if (m_infoClusterCirc[rootV].pos == -1) {
      m_infoClusterCirc[rootV].pos = varConnected.size();
      varConnected.push_back({});
      m_rootSetCirc.push_back(rootV);
    }
    varConnected[m_infoClusterCirc[rootV].pos].push_back(v);
  }

  for (auto r : m_rootSetCirc) m_infoClusterCirc[r].pos = -1;
  return varConnected.size();
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
  freeVar.clear();
  int nbComponent = 0;

  for (auto v : setOfVar) {
    if (varIsAssigned(v) || !m_isStillAlive[v] || m_idxComponentCirc[v] ||
        !isTargeted[v])
      continue;

    nbComponent++;
    m_idxComponentCirc[v] = nbComponent;
    m_tmpVecCirc.push_back(v);

    int cpt = 0;
    while (!m_tmpVecCirc.empty()) {
      cpt++;
      Var u = m_tmpVecCirc.back();
      m_tmpVecCirc.pop_back();

      for (unsigned gIdx : m_varGates[u]) {
        if (!isAliveGateIdx(gIdx)) continue;
        const BcGate& g = m_gates[gIdx];

        auto visitVar = [&](Var w) {
          if (varIsAssigned(w) || !m_isStillAlive[w] || m_idxComponentCirc[w])
            return;
          m_idxComponentCirc[w] = nbComponent;
          m_tmpVecCirc.push_back(w);
        };

        if (g.gateType != BcGateType::CLAUSE) visitVar(g.output.var());
        for (auto l : g.input) visitVar(l.var());
      }
    }

    if (cpt == 1) {
      m_idxComponentCirc[v] = 0;
      nbComponent--;
    }
  }

  varConnected.resize(nbComponent);
  for (auto v : setOfVar) {
    if (m_idxComponentCirc[v]) {
      assert(m_idxComponentCirc[v] <= (int)varConnected.size());
      varConnected[m_idxComponentCirc[v] - 1].push_back(v);
    } else if (!varIsAssigned(v) && m_isStillAlive[v]) {
      freeVar.push_back(v);
    }
    m_idxComponentCirc[v] = 0;
  }

  // Filter out no-longer-alive free variables (mirrors CircuitWithCnfManager).
  unsigned i = 0, j = 0;
  for (; i < freeVar.size(); i++)
    if (m_isStillAlive[freeVar[i]]) freeVar[j++] = freeVar[i];
  freeVar.resize(j);

  return nbComponent;
}

void CircuitManagerDyn::preUpdate(const std::vector<Lit>& lits) { exit(1); }
void CircuitManagerDyn::postUpdate(const std::vector<Lit>& lits) { exit(1); }

}  // namespace d4
