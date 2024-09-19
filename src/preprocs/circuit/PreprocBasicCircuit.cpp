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
#include "PreprocBasicCircuit.hpp"

#include "src/problem/ProblemManager.hpp"
#include "src/problem/circuit/ProblemManagerCircuit.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
 * @brief PreprocBasicCircuit::PreprocBasicCircuit implementation.
 */
PreprocBasicCircuit::PreprocBasicCircuit(std::ostream &out) {
  out << "c [PREPROC CIRCUIT] Basic\n";
}  // constructor

/**
 * @brief This preprocessing technique does nothing.
 *
 * @param[in] pin is the problem we want to prepocess.
 *
 * \return a copy of pin.
 */
ProblemManager *PreprocBasicCircuit::run(ProblemManager *pin,
                                         const OptionPreprocManager &option) {
  std::vector<Lit> units;
  return pin->getConditionedFormula(units);
}  // run

}  // namespace d4
