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

#include "../cnf/CnfManagerDyn.hpp"
#include "CircuitManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

class CircuitWithCnfManager : public CircuitManager {
 private:
  CnfManagerDyn *m_cnfManager;
  ProblemManagerCnf *m_problemCnf;

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

  void debugFunction();

 public:
  CircuitWithCnfManager(ProblemManager &p, bool optRmGates);
  ~CircuitWithCnfManager();

  inline CnfManager *getCnfManager() { return m_cnfManager; };
  inline bool isActiveGates(BcGate &g) override {
    return m_isStillAlive[g.output.var()];
  }

  bool stillActive(BcGate &g);
  void propagate(std::vector<Var> &vars, std::vector<Var> &pVars);

  int computeTrivialConnectedComponent(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<Var> &freeVar) override;

  /**
   * @brief Search for the connected component of the formula.
   *
   * @param[out] varConnected are the computed connected component.
   * @param setOfVar are the variables under consideration.
   * @param freeVar are the variables their are free (isFreeVar should return
   * true on them).
   *
   * @return the number of connected component.
   */
  int computeConnectedComponent(std::vector<std::vector<Var>> &varConnected,
                                std::vector<Var> &setOfVar,
                                std::vector<Var> &freeVar) override;

  /**
   * @brief Search for the connected component of the formula regarding a subset
   * of varaibles specified with a boolean vector.
   *
   * @param[out] varConnected are the computed connected component.
   * @param[in] setOfVar are the variables under consideration.
   * @param[in] isTargeted is a boolean vector that specifies the variable
   * under consideration.
   * @param[out] freeVar are the variables their are free (isFreeVar should
   * return true on them).
   *
   * @return the number of connected component.
   */
  int computeConnectedComponentTargeted(
      std::vector<std::vector<Var>> &varConnected, std::vector<Var> &setOfVar,
      std::vector<bool> &isTargeted, std::vector<Var> &freeVar) override;

  /**
   * @brief This function is called to update the formula regarding a set of
   * literals we want to assigned.
   *
   * @param[in] lits is the set of literal under consideration.
   */
  void preUpdate(const std::vector<Lit> &lits) override;

  /**
   * @brief This function is called to undo the modification done by the
   * function preUpdate.
   *
   * @param lits are the literals updated.
   */
  void postUpdate(const std::vector<Lit> &lits) override;

  /**
   * @brief Display the initial formula.
   *
   * @param[out] out is the stream used.
   */
  void showFormula(std::ostream &out) override;

  /**
   * @brief Display the current formula.
   *
   * @param[out] out is the stream used.
   */
  void showCurrentFormula(std::ostream &out) override;

  /**
   * @brief Display the current formula.
   *
   * @param[out] out is the stream used.
   */
  void showCurrentFormula(std::ostream &out,
                          std::vector<bool> &isInComponent) override;

  /**
   * @brief Get the problem input type.
   *
   * @return a element of the enum ProblemInputType.
   */
  ProblemInputType getProblemInputType() override;

  /**
   * @brief Display some information about the object.
   *
   * @param[out] out is the stream used.
   */
  void printInformation(std::ostream &out) override;

  /**
   * @brief Ask if the given variable is free in the current formula.
   *
   * @param[in] v is the variable we are looking for.
   *
   * @return true if the variable is free, false otherwise.
   */
  bool isFreeVariable(Var v) override;
};
}  // namespace d4