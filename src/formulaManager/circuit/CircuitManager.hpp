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

#include "../FormulaManager.hpp"
#include "../cnf/CnfManagerDyn.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

/**
 * @brief Shared base for all circuit managers.
 *
 * Holds the gate structure, the embedded CnfManagerDyn (used for SAT
 * propagation and formula storage), and the alive-gate tracking machinery
 * (watch lists, propagation, backtracking). Subclasses implement connected
 * component computation via different strategies.
 */
class CircuitManager : public FormulaManager {
 protected:
  std::vector<BcGate> m_gates;
  std::vector<Lit> m_true_lits;

  CnfManagerDyn* m_cnfManager;

  unsigned m_propagatedFree;
  unsigned m_lastIndex;
  std::vector<unsigned> m_varToGate;
  std::vector<Var> m_gateToVar;
  std::vector<std::vector<Var>> m_varInputInGates;
  std::vector<std::vector<Var>> m_watchList;
  std::vector<std::vector<Var>> m_litThatInactiveVar;

  bool m_optionRemoveGates;
  std::vector<bool> m_isStillAlive;
  std::vector<unsigned> m_stackGatesNotAlive;
  std::vector<unsigned> m_stackGatesNotAliveSize;

  /**
   * @brief Called whenever gate with output variable w is deactivated.
   * Subclasses override to maintain their own occurrence counts.
   */
  virtual void onGateDeactivated(Var w) {}

  /**
   * @brief Called whenever gate with output variable w is re-activated
   * during backtracking. Symmetric to onGateDeactivated.
   */
  virtual void onGateReactivated(Var w) {}

  void debugFunction();

 public:
  CircuitManager(const ProblemManager& p, bool optRmGates);
  ~CircuitManager() override;

  inline std::vector<BcGate>& getGates() { return m_gates; }
  inline CnfManager* getCnfManager() { return m_cnfManager; }
  inline ProblemInputType getProblemInputType() override { return PB_CIRC; }
  inline bool isActiveGates(BcGate& g) {
    return m_isStillAlive[g.output.var()];
  }

  bool stillActive(BcGate& g);
  void propagate(std::vector<Var>& vars, std::vector<Var>& pVars);

  int computeTrivialConnectedComponent(
      std::vector<std::vector<Var>>& varConnected, std::span<Var> setOfVar,
      std::vector<Var>& freeVar) override;

  void preUpdateGatesRemoved(const std::vector<Lit>& lits,
                             std::vector<Lit>& litsTrue);
  void postUpdateGatesRemoved(const std::vector<Lit>& lits);

  void showFormula(std::ostream& out) override;
  void showCurrentFormula(std::ostream& out) override;
  void showCurrentFormula(std::ostream& out,
                          std::vector<bool>& isInComponent) override;
  void printInformation(std::ostream& out) override;

  bool isFreeVariable(Var v) override;
  void initFormulaStore(const OptionBucketManager& opts) override;
  void storeFormula(std::span<const Var> component, DataBucket& b,
                    BucketAllocator& alloc) override;
};
}  // namespace d4
