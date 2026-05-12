/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include <vector>

#include "src/utils/Problem.hpp"

namespace bipe {

/**
 * @brief Defines the supported types of logical gates.
 *
 * These gate types represent structural functional dependencies that are
 * identified and extracted from a CNF formula.
 */
enum TypeGate {
  UNIT,   ///< A unit literal (acts as a constant true/false gate).
  EQUIV,  ///< An equivalence gate (e.g., output <-> input).
  OR,     ///< A logical OR gate.
  AND,    ///< A logical AND gate.
  XOR,    ///< A logical Exclusive OR (XOR) gate.
  RM      ///< Represents a gate that has been marked as removed or deactivated.
};

/**
 * @brief Represents a logical gate extracted from a formula.
 *
 * A gate expresses a functional dependency where the truth value of an
 * `output` literal is strictly defined by a specific logical operation
 * applied to a set of `input` literals.
 */
struct Gate {
  TypeGate type;  ///< The logical operation performed by this gate.
  Lit output;     ///< The literal that serves as the output of the gate.
  std::vector<Lit> input;  ///< The collection of literals acting as inputs.

  /**
   * @brief Prints a human-readable representation of the gate to standard
   * output.
   *
   * Formats the gate depending on its type (e.g., "3 <-> AND( 1 2 )").
   */
  inline void display() {
    std::cout << output << " <-> ";

    switch (type) {
      case UNIT:
        std::cout << "(T ";
        break;
      case EQUIV:
        std::cout << "( ";
        break;
      case OR:
        std::cout << "OR( ";
        break;
      case AND:
        std::cout << "AND( ";
        break;
      case XOR:
        std::cout << "XOR( ";
        break;
      default:
        break;
    }

    for (auto l : input) std::cout << l << " ";
    std::cout << ")\n";
  }
};

}  // namespace bipe