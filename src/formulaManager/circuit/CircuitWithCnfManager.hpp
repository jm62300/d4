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

#include "CircuitManager.hpp"
#include "src/formulaManager/cnf/CnfManagerDyn.hpp"

namespace d4 {

/**
 * @brief Circuit manager that delegates connected component computation to the
 * embedded CnfManagerDyn (via the Tseitin expansion of the circuit).
 */
class CircuitWithCnfManager : public CircuitManager, public CnfManagerDyn {
 private:
  std::vector<Lit> m_litsTrue;

  static ProblemManager makeTseitinProblem(const ProblemManager& p);

 public:
  CircuitWithCnfManager(const ProblemManager& p, bool optRmGates);

  inline ProblemInputType getProblemInputType() override { return PB_CIRC; }

  int computeConnectedComponent(std::vector<std::vector<Var>>& varConnected,
                                std::span<Var> setOfVar,
                                std::vector<Var>& freeVar) override;

  int computeConnectedComponentTargeted(
      std::vector<std::vector<Var>>& varConnected, std::vector<Var>& setOfVar,
      std::vector<bool>& isTargeted, std::vector<Var>& freeVar) override;

  void preUpdate(const std::vector<Lit>& lits) override;
  void postUpdate(const std::vector<Lit>& lits) override;

  void showFormula(std::ostream& out) override;
  void showCurrentFormula(std::ostream& out) override;
  void showCurrentFormula(std::ostream& out,
                          std::vector<bool>& isInComponent) override;
  void printInformation(std::ostream& out) override;
  bool isFreeVariable(Var v) override;
  void initFormulaStore(const OptionBucketManager& opts) override;
  void storeFormula(std::span<const Var> component, DataBucket& b,
                    BucketAllocator& alloc) override;
  CnfManager* getCnfManager() override { return this; }
};
}  // namespace d4
