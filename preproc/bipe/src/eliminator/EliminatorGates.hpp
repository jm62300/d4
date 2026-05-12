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

#include "src/utils/Gate.hpp"
#include "src/utils/ProblemTypes.hpp"
#include "src/utils/Timer.hpp"

namespace bipe {
namespace eliminator {

/**
 * @brief Applies variable elimination using gate extraction.
 *
 * This class attempts to simplify a CNF formula by identifying logic gates
 * (such as OR, XOR, and Equivalence) represented as a Directed Acyclic Circuit
 * (DAC). It then eliminates variables by substituting these gates, provided the
 * elimination does not exceed a specified threshold for clause growth.
 */
class EliminatorGates {
 private:
  std::vector<bool> m_marked;
  std::vector<bool> m_markedCanBeUsed;
  std::vector<bool> m_isUnit;

  static const unsigned LIMIT_XOR_SIZE = 5;
  Lit m_memory_xor[1 << LIMIT_XOR_SIZE][5];
  unsigned m_lookupTable[1 << LIMIT_XOR_SIZE];

  /**
   * @brief Applies Boolean Constraint Propagation (BCP) on clauses and the DAC
   * using unit literals.
   *
   * @param units The current set of unit literals to propagate.
   * @param clauses The set of clauses in the formula.
   * @param occClauses The adjacency list mapping literals/variables to their
   * clauses.
   * @param freePlace A stack tracking the indices of removed clauses for memory
   * reuse.
   * @param dac The Directed Acyclic Circuit representing extracted gates.
   * @param occDac The adjacency list mapping variables to their associated
   * gates in the DAC.
   * @param parentCounter Tracks how many times each variable acts as a parent
   * in the DAC.
   * @param eliminated The list collecting successfully eliminated literals.
   *
   * @return False if the problem is proven unsatisfiable during propagation,
   * true otherwise.
   */
  bool removeUnit(std::vector<Lit>& units,
                  std::vector<std::vector<Lit>>& clauses,
                  std::vector<std::vector<unsigned>>& occClauses,
                  std::vector<unsigned>& freePlace, std::vector<Gate>& dac,
                  std::vector<std::vector<unsigned>>& occDac,
                  std::vector<unsigned>& parentCounter,
                  std::vector<Lit>& eliminated);

  /**
   * @brief Eliminates a variable defined by an equivalence gate.
   *
   * @param units The current set of unit literals.
   * @param clauses The set of clauses in the formula.
   * @param occClauses The adjacency list mapping variables to their clauses.
   * @param freePlace A stack tracking the indices of removed clauses.
   * @param g The equivalence gate used to eliminate the variable.
   * @param idxGate The index of the gate within the DAC.
   * @param occDac The adjacency list for the DAC.
   * @param parentCounter Tracks how many times each variable acts as a parent.
   * @param eliminated The list collecting successfully eliminated literals.
   */
  void eliminateOneEquiv(std::vector<Lit>& units,
                         std::vector<std::vector<Lit>>& clauses,
                         std::vector<std::vector<unsigned>>& occClauses,
                         std::vector<unsigned>& freePlace, Gate& g,
                         unsigned idxGate,
                         std::vector<std::vector<unsigned>>& occDac,
                         std::vector<unsigned>& parentCounter,
                         std::vector<Lit>& eliminated);

  /**
   * @brief Eliminates a variable defined by an OR gate.
   *
   * @param units The current set of unit literals.
   * @param clauses The set of clauses in the formula.
   * @param occClauses The adjacency list mapping variables to their clauses.
   * @param freePlace A stack tracking the indices of removed clauses.
   * @param g The OR gate used to eliminate the variable.
   * @param idxGate The index of the gate within the DAC.
   * @param occDac The adjacency list for the DAC.
   * @param parentCounter Tracks how many times each variable acts as a parent.
   * @param eliminated The list collecting successfully eliminated literals.
   */
  void eliminateOneOr(std::vector<Lit>& units,
                      std::vector<std::vector<Lit>>& clauses,
                      std::vector<std::vector<unsigned>>& occClauses,
                      std::vector<unsigned>& freePlace, Gate& g,
                      unsigned idxGate,
                      std::vector<std::vector<unsigned>>& occDac,
                      std::vector<unsigned>& parentCounter,
                      std::vector<Lit>& eliminated);

  /**
   * @brief Eliminates a variable defined by an XOR gate.
   *
   * @param units The current set of unit literals.
   * @param clauses The set of clauses in the formula.
   * @param occClauses The adjacency list mapping variables to their clauses.
   * @param freePlace A stack tracking the indices of removed clauses.
   * @param g The XOR gate used to eliminate the variable.
   * @param idxGate The index of the gate within the DAC.
   * @param occDac The adjacency list for the DAC.
   * @param parentCounter Tracks how many times each variable acts as a parent.
   * @param eliminated The list collecting successfully eliminated literals.
   */
  void eliminateOneXor(std::vector<Lit>& units,
                       std::vector<std::vector<Lit>>& clauses,
                       std::vector<std::vector<unsigned>>& occClauses,
                       std::vector<unsigned>& freePlace, Gate& g,
                       unsigned idxGate,
                       std::vector<std::vector<unsigned>>& occDac,
                       std::vector<unsigned>& parentCounter,
                       std::vector<Lit>& eliminated);

  /**
   * @brief Attempts to eliminate a single variable using one of its defining
   * gates.
   *
   * @param units The current set of unit literals (typically empty at the
   * beginning).
   * @param clauses The set of clauses in the formula.
   * @param occClauses The adjacency list mapping variables to their clauses.
   * @param freePlace A stack tracking the indices of removed clauses.
   * @param dac The Directed Acyclic Circuit.
   * @param occDac The adjacency list for the DAC.
   * @param parentCounter Tracks how many times a variable acts as a parent.
   * @param eliminated The list collecting successfully eliminated literals.
   * @param limitNbClauses The maximum permitted total number of clauses after
   * elimination.
   * @return True if at least one variable was successfully eliminated, false
   * otherwise.
   */
  bool eliminateOneGate(std::vector<Lit>& units,
                        std::vector<std::vector<Lit>>& clauses,
                        std::vector<std::vector<unsigned>>& occClauses,
                        std::vector<unsigned>& freePlace,
                        std::vector<Gate>& dac,
                        std::vector<std::vector<unsigned>>& occDac,
                        std::vector<unsigned>& parentCounter,
                        std::vector<Lit>& eliminated, unsigned limitNbClauses);

  /**
   * @brief Searches for a specific element in an occurrence list and removes
   * it.
   *
   * Uses a fast swap-and-pop technique to avoid shifting elements.
   *
   * @tparam T The type of the elements in the list.
   * @param occ The list from which the element will be removed.
   * @param idx The element to find and remove.
   */
  template <class T>
  inline void removeOcc(std::vector<T>& occ, T idx) {
    unsigned pos = 0;
    while (pos < occ.size() && occ[pos] != idx) pos++;
    assert(pos < occ.size());
    occ[pos] = occ.back();
    occ.pop_back();
  }

  /**
   * @brief Evaluates whether a gate can be safely used to eliminate a variable.
   *
   * Checks if eliminating the variable via this gate complies with the
   * specified maximum clause limit.
   *
   * @param g The gate being evaluated.
   * @param clauses The set of clauses in the formula.
   * @param freePlace The list of currently available clause indices.
   * @param occClauses The adjacency list mapping variables to clauses.
   * @param limitNbClauses The maximum permitted number of clauses after
   * replacing the variable.
   * @return True if the gate can be used without exceeding limits, false
   * otherwise.
   */
  bool canBeUsed(Gate& g, std::vector<std::vector<Lit>>& clauses,
                 std::vector<unsigned>& freePlace,
                 std::vector<std::vector<unsigned>>& occClauses,
                 unsigned limitNbClauses);

 public:
  /**
   * @brief Constructs a new EliminatorGates object.
   *
   * Initializations and linkage to external structures or other eliminators
   * are handled here.
   */
  EliminatorGates();

  /**
   * @brief Eliminates variables from the given formula utilizing the provided
   * DAC.
   *
   * Iteratively searches for removable gates within the Directed Acyclic
   * Circuit and simplifies the CNF formula. Assumes the provided problem is
   * satisfiable.
   *
   * @param nbVar The total number of variables in the formula.
   * @param[in,out] clauses The formula to be simplified in-place.
   * @param dac The Directed Acyclic Circuit containing the extracted logic
   * gates.
   * @param[out] eliminated The list where successfully eliminated literals are
   * stored.
   * @param verbose If true, outputs detailed logs of the elimination progress.
   * @param limitNbClauses The maximum permitted total number of clauses during
   * the process.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void eliminate(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
                 std::vector<Gate>& dac, std::vector<Lit>& eliminated,
                 bool verbose, unsigned limitNbClauses, const Timer& timer);
};

}  // namespace eliminator
}  // namespace bipe