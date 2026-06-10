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

namespace d4 {

CircuitWithCnfManager::CircuitWithCnfManager(const ProblemManager& p,
                                             bool optRmGates)
    : CircuitManager(p, optRmGates) {}

CircuitWithCnfManager::~CircuitWithCnfManager() {}

/**
 * @brief Compute connected components by delegating to the embedded
 * CnfManagerDyn, then filter out no-longer-alive free variables.
 */
int CircuitWithCnfManager::computeConnectedComponent(
    std::vector<std::vector<Var>>& varConnected, std::span<Var> setOfVar,
    std::vector<Var>& freeVar) {
  int ret =
      m_cnfManager->computeConnectedComponent(varConnected, setOfVar, freeVar);

  unsigned i, j;
  for (i = j = 0; i < freeVar.size(); i++)
    if (m_isStillAlive[freeVar[i]]) freeVar[j++] = freeVar[i];
  freeVar.resize(j);

  return ret;
}

/**
 * @brief Targeted CC by delegating to the embedded CnfManagerDyn, then filter
 * out no-longer-alive free variables.
 */
int CircuitWithCnfManager::computeConnectedComponentTargeted(
    std::vector<std::vector<Var>>& varConnected, std::vector<Var>& setOfVar,
    std::vector<bool>& isTargeted, std::vector<Var>& freeVar) {
  int ret = m_cnfManager->computeConnectedComponentTargeted(
      varConnected, setOfVar, isTargeted, freeVar);

  unsigned i, j;
  for (i = j = 0; i < freeVar.size(); i++)
    if (m_isStillAlive[freeVar[i]]) freeVar[j++] = freeVar[i];
  freeVar.resize(j);

  return ret;
}  // computeConnectedComponentTargeted

/**
 * @brief CircuitWithCnfManager::preUpdate
 */
void CircuitWithCnfManager::preUpdate(const std::vector<Lit>& lits) {
  m_cnfManager->pushStacks();
  assignListLit(lits);
  m_cnfManager->assignListLit(lits);

  preUpdateGatesRemoved(lits, m_litsTrue);
  m_cnfManager->propagateTrue(m_litsTrue);
  m_cnfManager->propagateFalseInNotBin(lits);
  m_cnfManager->unmarkLastClausesSaved();
}  // preUpdate

/**
 * @brief CircuitWithCnfManager::postUpdate
 */
void CircuitWithCnfManager::postUpdate(const std::vector<Lit>& lits) {
  for (auto& l : lits) m_currentValue[l.var()] = l_Undef;
  postUpdateGatesRemoved(lits);
  m_cnfManager->postUpdate(lits);
}  // postUpdate

}  // namespace d4
