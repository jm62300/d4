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
class CircuitManagerDyn : public CircuitManager, public CnfManagerDyn {
 private:
  static ProblemManager onlyWithClauseProblem(const ProblemManager& p);

 public:
  CircuitManagerDyn(const ProblemManager& p, bool optRmGates);
  ~CircuitManagerDyn() override;

  int computeConnectedComponent(std::vector<std::vector<Var>>& varConnected,
                                std::span<Var> setOfVar,
                                std::vector<Var>& freeVar) override;

  int computeConnectedComponentTargeted(
      std::vector<std::vector<Var>>& varConnected, std::vector<Var>& setOfVar,
      std::vector<bool>& isTargeted, std::vector<Var>& freeVar) override;

  void preUpdate(const std::vector<Lit>& lits) override;
  void postUpdate(const std::vector<Lit>& lits) override;
  CnfManager* getCnfManager() override { return this; }
  inline ProblemInputType getProblemInputType() override { return PB_CIRC; }
};
}  // namespace d4
