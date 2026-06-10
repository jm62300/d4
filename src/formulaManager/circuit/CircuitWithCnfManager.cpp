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

ProblemManager CircuitWithCnfManager::makeTseitinProblem(
    const ProblemManager& p) {
  std::vector<BcGate> tseitinGates;
  p.toCnf(tseitinGates);
  return ProblemManager("cnf", p.getNbVar(), p.getQuantification(),
                        p.getWeightMap(), tseitinGates, std::cout);
}

CircuitWithCnfManager::CircuitWithCnfManager(const ProblemManager& p,
                                             bool optRmGates)
    : FormulaManager(p.getNbVar()),
      CircuitManager(p, optRmGates),
      CnfManagerDyn(makeTseitinProblem(p)) {
  if (m_activeLitTrue.size()) {
    pushStacks();
    propagateTrue(m_activeLitTrue);
    unmarkLastClausesSaved();
  }
}  // constructor

int CircuitWithCnfManager::computeConnectedComponent(
    std::vector<std::vector<Var>>& varConnected, std::span<Var> setOfVar,
    std::vector<Var>& freeVar) {
  int ret = CnfManager::computeConnectedComponent(varConnected, setOfVar,
                                                  freeVar);
  unsigned i, j;
  for (i = j = 0; i < freeVar.size(); i++)
    if (isStillAliveVar(freeVar[i])) freeVar[j++] = freeVar[i];
  freeVar.resize(j);
  return ret;
}  // computeConnectedComponent

int CircuitWithCnfManager::computeConnectedComponentTargeted(
    std::vector<std::vector<Var>>& varConnected, std::vector<Var>& setOfVar,
    std::vector<bool>& isTargeted, std::vector<Var>& freeVar) {
  int ret = CnfManager::computeConnectedComponentTargeted(
      varConnected, setOfVar, isTargeted, freeVar);
  if (!m_optionRemoveGates) return ret;

  unsigned i, j;
  for (i = j = 0; i < freeVar.size(); i++)
    if (isStillAliveVar(freeVar[i])) freeVar[j++] = freeVar[i];
  freeVar.resize(j);
  return ret;
}  // computeConnectedComponentTargeted

void CircuitWithCnfManager::preUpdate(const std::vector<Lit>& lits) {
  pushStacks();
  assignListLit(lits);
  preUpdateGatesRemoved(lits, m_litsTrue);
  propagateTrue(m_litsTrue);
  propagateFalseInNotBin(lits);
  unmarkLastClausesSaved();
}  // preUpdate

void CircuitWithCnfManager::postUpdate(const std::vector<Lit>& lits) {
  for (auto& l : lits) m_currentValue[l.var()] = l_Undef;
  postUpdateGatesRemoved(lits);
  CnfManagerDyn::postUpdate(lits);
}  // postUpdate

void CircuitWithCnfManager::showFormula(std::ostream& out) {
  CnfManager::showFormula(out);
}  // showFormula

void CircuitWithCnfManager::showCurrentFormula(std::ostream& out) {
  CnfManager::showCurrentFormula(out);
}  // showCurrentFormula

void CircuitWithCnfManager::showCurrentFormula(
    std::ostream& out, std::vector<bool>& isInComponent) {
  CnfManager::showCurrentFormula(out, isInComponent);
}  // showCurrentFormula

void CircuitWithCnfManager::printInformation(std::ostream& out) {
  out << "c \033[1m\033[36mFormula Manager Information\033[0m\n";
  CnfManager::printInformation(out);
  out << "c Number of variable eliminated: " << m_propagatedFree << '\n';
  out << "c\n";
}  // printInformation

bool CircuitWithCnfManager::isFreeVariable(Var v) {
  return CnfManager::isFreeVariable(v);
}  // isFreeVariable

void CircuitWithCnfManager::initFormulaStore(const OptionBucketManager& opts) {
  CnfManager::initFormulaStore(opts);
}  // initFormulaStore

void CircuitWithCnfManager::storeFormula(std::span<const Var> component,
                                         DataBucket& b,
                                         BucketAllocator& alloc) {
  CnfManager::storeFormula(component, b, alloc);
}  // storeFormula

}  // namespace d4
