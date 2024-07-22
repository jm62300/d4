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

#include <ostream>

#include "../PartialOrderHeuristicTreeDecomp.hpp"
#include "src/formulaManager/circuit/CircuitWithCnfManager.hpp"

namespace d4 {
class PartialOrderHeuristicTreeDecompCircuit
    : public PartialOrderHeuristicTreeDecomp {
 public:
  /**
   * @brief Constructor.
   *
   * @param options gives the list of options.
   * @param om is the object which deal with the formula.
   * @param s is a SAT solver associate with the formula under consideration.
   * @param out is the output stream.
   */
  PartialOrderHeuristicTreeDecompCircuit(
      const OptionPartialOrderHeuristic &options, CircuitManager &om,
      WrapperSolver &s, std::ostream &out);

  ~PartialOrderHeuristicTreeDecompCircuit();
};
}  // namespace d4
