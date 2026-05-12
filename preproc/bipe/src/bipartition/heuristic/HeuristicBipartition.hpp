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

#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {

/**
 * @brief Base class and factory for bipartition heuristics.
 *
 * This class defines the interface for heuristic strategies used to determine
 * the order in which variables are tested during the bipartitioning process.
 * It manages a list of assumption literals that represent the variables
 * still waiting to be evaluated.
 */
class HeuristicBipartition {
 protected:
  /// The current list of assumptions (literals) to be tested.
  std::vector<Lit> m_assumptions;

 public:
  /**
   * @brief Virtual destructor to ensure proper cleanup of derived heuristic
   * classes.
   */
  virtual ~HeuristicBipartition() = default;

  /**
   * @brief Factory method to instantiate a specific bipartition heuristic.
   *
   * @param p The problem instance for which we are computing the bipartition.
   * @param selectors The set of selector literals (representing a one-to-one
   * mapping with the initial variables) used to toggle equivalences.
   * @param method The string identifier of the specific heuristic to create
   *               (e.g., "occurrence", "random").
   * @param out The output stream used for displaying initialization logs.
   * @return A pointer to the dynamically allocated heuristic instance.
   */
  static HeuristicBipartition* makeHeuristicBipartition(
      Problem& p, const std::vector<Lit>& selectors, const std::string& method,
      std::ostream& out);

  /**
   * @brief Retrieves the list of remaining assumptions waiting to be tested.
   * @return A reference to the internal vector of assumption literals.
   */
  inline std::vector<Lit>& remainingAssumptions() { return m_assumptions; }

  /**
   * @brief Checks if there are no more assumptions left to test.
   * @return True if the assumption list is empty, false otherwise.
   */
  inline bool isEmpty() { return !m_assumptions.size(); }

  /**
   * @brief Removes a specific variable from the assumption list.
   *
   * Iterates through the current assumptions and safely removes any literal
   * that corresponds to the given variable without changing the order of the
   * remaining elements.
   *
   * @param v The variable to remove from the assumptions.
   */
  inline void popVarFromAssumption(Var v) {
    unsigned j = 0;
    for (unsigned i = 0; i < m_assumptions.size(); i++)
      if (m_assumptions[i].var() != v) m_assumptions[j++] = m_assumptions[i];
    m_assumptions.resize(j);
  }

  /**
   * @brief Overwrites the current assumption list with a new set of
   * assumptions.
   * @param assumptions The new vector of literals to set as assumptions.
   */
  inline void setAssumption(std::vector<Lit>& assumptions) {
    m_assumptions = assumptions;
  }

  /**
   * @brief Retrieves the next assumption to test and removes it from the list.
   *
   * By default, this implementation treats the assumption list as a stack,
   * popping and returning the literal at the back. Derived classes can override
   * this to implement more complex selection logic.
   *
   * @return The chosen assumption literal.
   */
  virtual Lit nextAssumption() {
    Lit ret = m_assumptions.back();
    m_assumptions.pop_back();
    return ret;
  }
};

}  // namespace bipartition
}  // namespace bipe