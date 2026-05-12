/**
 * eliminator
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

#include <iostream>
#include <vector>

#include "EliminatorGates.hpp"
#include "EliminatorResolution.hpp"
#include "src/utils/ProblemTypes.hpp"
#include "src/utils/Timer.hpp"

namespace bipe {
namespace eliminator {

/**
 * @brief Orchestrates the variable elimination process for a CNF formula.
 *
 * This is the main manager class for Bounded Variable Elimination (BVE).
 * It combines both gate-based elimination (utilizing a Directed Acyclic
 * Circuit) and resolution-based elimination to iteratively simplify the formula
 * while strictly protecting designated input variables.
 */
class Eliminator {
 private:
  bool m_isInterrupted = false;
  std::vector<bool> m_marked;
  std::vector<bool> m_isUnit;

  EliminatorGates m_elimGates;
  EliminatorResolution m_elimResolution;

 public:
  /**
   * @brief Constructs a new Eliminator object.
   *
   * Initializes the internal elimination engines (Gates and Resolution).
   */
  Eliminator();

  /**
   * @brief Attempts to simplify a satisfiable CNF formula by eliminating
   * non-input variables.
   *
   * The method alternates between gate-based and resolution-based elimination
   * to remove as many variables as possible without exceeding the clause growth
   * limit.
   *
   * @param nbVar The total number of variables in the formula.
   * @param[in,out] clauses The CNF formula to be processed and modified
   * in-place.
   * @param input The set of protected input variables that must NOT be
   * eliminated.
   * @param dac The Directed Acyclic Circuit (DAC) used for gate-based
   * elimination.
   * @param[out] eliminated The list where successfully eliminated literals are
   * stored.
   * @param verbose If true, prints detailed progress logs to the standard
   * output.
   * @param limitNbClauses The maximum allowed total number of clauses permitted
   *                       during the elimination process.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void eliminate(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
                 std::vector<Var>& input, std::vector<Gate>& dac,
                 std::vector<Lit>& eliminated, bool verbose,
                 unsigned limitNbClauses, const Timer& timer);
};

}  // namespace eliminator
}  // namespace bipe