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

namespace d4 {

/**
 * @brief Circuit manager that delegates connected component computation to the
 * embedded CnfManagerDyn (via the Tseitin expansion of the circuit).
 */
class CircuitWithCnfManager : public CircuitManager {
 private:
  std::vector<Lit> m_litsTrue;

 public:
  CircuitWithCnfManager(const ProblemManager& p, bool optRmGates);
  ~CircuitWithCnfManager() override;

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
