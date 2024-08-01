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
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"

namespace d4 {

class CircuitManager : public FormulaManager {
 protected:
  std::vector<BcGate> m_gates;
  std::vector<Lit> m_true_lits;

  CircuitManager(ProblemManager &p);

 public:
  virtual ~CircuitManager() {}

  inline std::vector<BcGate> &getGates() { return m_gates; }
  inline ProblemInputType getProblemInputType() override { return PB_CIRC; }
  virtual bool isActiveGates(BcGate &g) = 0;
};
}  // namespace d4