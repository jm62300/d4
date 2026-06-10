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

#include "../cnf/CnfManager.hpp"
#include "CircuitManager.hpp"

namespace d4 {

/**
 * @brief Circuit manager that computes connected components directly from the
 * gate hypergraph, without going through the Tseitin CNF expansion.
 *
 * Uses a union-find algorithm over gate indices, mirroring
 * CnfManager::computeConnectedComponent but with gates playing the role of
 * clauses.  Maintains per-variable alive-gate counts through the virtual hooks
 * onGateDeactivated / onGateReactivated.
 */
class CircuitManagerDyn : public CircuitManager {
 private:
  // Gate occurrence lists: m_varGates[v] = indices of gates where v appears
  // (as output for non-CLAUSE gates, or as an input for CLAUSE gates and also
  //  as an input for non-CLAUSE gates).
  std::vector<std::vector<unsigned>> m_varGates;

  // Count of currently alive gate occurrences per variable.
  // Decremented by onGateDeactivated; incremented by onGateReactivated.
  std::vector<unsigned> m_nbActiveGates;

  // Union-find scratch structures (mirror of CnfManager).
  // Size: nbVar + nbGates + 1  (gate gIdx occupies slot gIdx + nbVar + 1).
  std::vector<InfoCluster> m_infoClusterCirc;
  Var* m_activeVarsCirc;
  std::vector<Var> m_rootSetCirc;

  // Stamp-based marking of visited gates (mirrors m_stampMarkView/m_markView).
  uint32_t m_stampCirc;
  std::vector<uint32_t> m_markCirc;  // indexed by gate index

  // Auxiliary structures for the targeted CC (BFS variant).
  std::vector<int> m_idxComponentCirc;
  std::vector<Var> m_tmpVecCirc;

  inline bool isAliveGateIdx(unsigned idx);
  inline void incrementStampCirc();

 public:
  CircuitManagerDyn(const ProblemManager& p, bool optRmGates);
  ~CircuitManagerDyn() override;

  bool isFreeVariable(Var v) override;

  int computeConnectedComponent(std::vector<std::vector<Var>>& varConnected,
                                std::span<Var> setOfVar,
                                std::vector<Var>& freeVar) override;

  int computeConnectedComponentTargeted(
      std::vector<std::vector<Var>>& varConnected, std::vector<Var>& setOfVar,
      std::vector<bool>& isTargeted, std::vector<Var>& freeVar) override;

  void preUpdate(const std::vector<Lit>& lits) override;
  void postUpdate(const std::vector<Lit>& lits) override;
};
}  // namespace d4
