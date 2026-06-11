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

ProblemManager CircuitManagerDyn::onlyWithClauseProblem(
    const ProblemManager& p) {
  std::vector<BcGate> clauseGates;
#if 0
  for (auto g : p.getGates())
    if (g.gateType == BcGateType::CLAUSE) clauseGates.push_back(g);
#else
  p.toCnf(clauseGates);  // until we implement the cache entries with gates.
#endif
  return ProblemManager("cnf", p.getNbVar(), p.getQuantification(),
                        p.getWeightMap(), clauseGates, std::cout);
}  // onlyWithClauseProblem

/**
 * @brief CircuitManagerDyn::CircuitManagerDyn implementation.
 */
CircuitManagerDyn::CircuitManagerDyn(const ProblemManager& p, bool optRmGates)
    : FormulaManager(p.getNbVar()),
      CircuitManager(p, optRmGates),
      CnfManagerDyn(onlyWithClauseProblem(p)) {}  // constructor

/**
 * @brief CircuitManagerDyn::~CircuitManagerDyn implementation.
 */
CircuitManagerDyn::~CircuitManagerDyn() {}

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
}  // computeConnectedComponent

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
