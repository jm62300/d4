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

/**
 * @brief CircuitWithCnfManager::CircuitWithCnfManager implementation.
 */
CircuitWithCnfManager::CircuitWithCnfManager(const ProblemManager& p,
                                             bool optRmGates)
    : CircuitManager(p, optRmGates) {
  std::vector<BcGate> tseitinGates;
  p.toCnf(tseitinGates);
  ProblemManager tseitinProblem("cnf", p.getNbVar(), p.getQuantification(),
                                p.getWeightMap(), tseitinGates, std::cout);
  m_cnfManager = new CnfManagerDyn(tseitinProblem);

  if (m_activeLitTrue.size()) {
    m_cnfManager->pushStacks();
    m_cnfManager->propagateTrue(m_activeLitTrue);
    m_cnfManager->unmarkLastClausesSaved();
  }
}  // constructor

/**
 * @brief CircuitWithCnfManager::~CircuitWithCnfManager implementation.
 */
CircuitWithCnfManager::~CircuitWithCnfManager() { delete m_cnfManager; }

/**
 * @brief CircuitWithCnfManager::computeConnectedComponent implementation.
 */
int CircuitWithCnfManager::computeConnectedComponent(
    std::vector<std::vector<Var>>& varConnected, std::span<Var> setOfVar,
    std::vector<Var>& freeVar) {
  int ret =
      m_cnfManager->computeConnectedComponent(varConnected, setOfVar, freeVar);

  unsigned i, j;
  for (i = j = 0; i < freeVar.size(); i++)
    if (isStillAliveVar(freeVar[i])) freeVar[j++] = freeVar[i];
  freeVar.resize(j);

  return ret;
}  // computeConnectedComponent

/**
 * @brief Targeted CC by delegating to the embedded CnfManagerDyn, then filter
 * out no-longer-alive free variables.
 */
int CircuitWithCnfManager::computeConnectedComponentTargeted(
    std::vector<std::vector<Var>>& varConnected, std::vector<Var>& setOfVar,
    std::vector<bool>& isTargeted, std::vector<Var>& freeVar) {
  int ret = m_cnfManager->computeConnectedComponentTargeted(
      varConnected, setOfVar, isTargeted, freeVar);
  if (!m_optionRemoveGates) return ret;

  // check if we remove
  unsigned i, j;
  for (i = j = 0; i < freeVar.size(); i++)
    if (isStillAliveVar(freeVar[i])) freeVar[j++] = freeVar[i];
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

/**
 * @brief CircuitWithCnfManager::showFormula implementation.
 */
void CircuitWithCnfManager::showFormula(std::ostream& out) {
  m_cnfManager->showFormula(out);
}  // showFormula

/**
 * @brief CircuitWithCnfManager::showCurrentFormula implementation.
 */
void CircuitWithCnfManager::showCurrentFormula(std::ostream& out) {
  m_cnfManager->showCurrentFormula(out);
}  // showCurrentFormula

/**
 * @brief CircuitWithCnfManager::showCurrentFormula (with component mask)
 * implementation.
 */
void CircuitWithCnfManager::showCurrentFormula(
    std::ostream& out, std::vector<bool>& isInComponent) {
  m_cnfManager->showCurrentFormula(out, isInComponent);
}  // showCurrentFormula

/**
 * @brief CircuitWithCnfManager::printInformation implementation.
 */
void CircuitWithCnfManager::printInformation(std::ostream& out) {
  out << "c \033[1m\033[36mFormula Manager Information\033[0m\n";
  m_cnfManager->printInformation(out);
  out << "c Number of variable eliminated: " << m_propagatedFree << '\n';
  out << "c\n";
}  // printInformation

/**
 * @brief CircuitWithCnfManager::isFreeVariable implementation.
 * Default: delegates to the embedded CNF manager.
 */
bool CircuitWithCnfManager::isFreeVariable(Var v) {
  return m_cnfManager->isFreeVariable(v);
}  // isFreeVariable

/**
 * @brief CircuitManager::initFormulaStore implementation.
 */
void CircuitWithCnfManager::initFormulaStore(const OptionBucketManager& opts) {
  m_cnfManager->initFormulaStore(opts);
}  // initFormulaStore

/**
 * @brief CircuitWithCnfManager::storeFormula implementation.
 */
void CircuitWithCnfManager::storeFormula(std::span<const Var> component,
                                         DataBucket& b,
                                         BucketAllocator& alloc) {
  m_cnfManager->storeFormula(component, b, alloc);
}  // storeFormula

}  // namespace d4
