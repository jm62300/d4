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

  unsigned m_propagatedFree;
  unsigned m_lastIndex;
  std::vector<unsigned> m_varToGate;
  std::vector<Var> m_gateToVar;
  std::vector<std::vector<Var>> m_varInputInGates;
  std::vector<std::vector<Var>> m_watchList;
  std::vector<std::vector<Var>> m_litThatInactiveVar;

  bool m_optionRemoveGates;
  std::vector<Lit> m_activeLitTrue;

 private:
  std::vector<bool> m_isStillAlive;
  std::vector<unsigned> m_stackGatesNotAlive;
  std::vector<unsigned> m_stackGatesNotAliveSize;

  void debugFunction();

 public:
  /**
   * @brief Init the CircuitManager in order to make possible the suppresion of
   * unactive gates. At the end of the initialization it could be possible that
   * some literals are assigned to true (the variables they are deactivated
   * during the process, which means that the both sides are assigned to true).
   *
   * @param p is the problem we want to handle.
   * @param optRmGates is set to true if the option that make possible the
   * suppresion of gates is activated.
   */
  CircuitManager(const ProblemManager& p, bool optRmGates);

  inline std::vector<BcGate>& getGates() { return m_gates; }
  inline ProblemInputType getProblemInputType() override { return PB_CIRC; }
  inline bool isActiveGates(BcGate& g) {
    return !m_optionRemoveGates || m_isStillAlive[g.output.var()];
  }

  inline bool isStillAliveVar(Var v) {
    return !m_optionRemoveGates || m_isStillAlive[v];
  }

  bool stillActive(BcGate& g);
  void propagate(std::vector<Var>& vars, std::vector<Var>& pVars);

  int computeTrivialConnectedComponent(
      std::vector<std::vector<Var>>& varConnected, std::span<Var> setOfVar,
      std::vector<Var>& freeVar) override;

  void preUpdateGatesRemoved(const std::vector<Lit>& lits,
                             std::vector<Lit>& litsTrue);
  void postUpdateGatesRemoved(const std::vector<Lit>& lits);
  virtual CnfManager* getCnfManager() = 0;
};
}  // namespace d4
