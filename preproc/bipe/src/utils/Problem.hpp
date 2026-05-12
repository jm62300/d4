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

#include "src/utils/ProblemTypes.hpp"
namespace bipe {

/**
 * @brief Represents a Boolean Satisfiability (SAT) problem in Conjunctive
 * Normal Form (CNF).
 *
 * This class encapsulates a CNF formula along with metadata essential for
 * advanced SAT tasks like model counting and bipartitioning. It tracks the
 * total number of variables, the set of clauses, and specific subsets of
 * variables such as projected variables (for projected model counting) and
 * protected variables (which should not be eliminated during preprocessing).
 */
class Problem {
 private:
  std::vector<std::vector<Lit>> m_clauses;
  std::vector<Var> m_projected;
  std::vector<Var> m_protected;
  std::vector<Var> m_heapVariable;
  unsigned m_nbVar;

 public:
  /**
   * @brief Constructs an empty formula with 0 variables and no clauses.
   */
  Problem();

  /**
   * @brief Constructs a new problem from raw integer representations.
   *
   * @param nbVar The total number of variables in the formula.
   * @param clauses The CNF clauses, where each inner vector is a clause of
   * integer literals.
   * @param projected The list of projected variables (integer representation).
   * @param protect The list of protected variables (integer representation).
   */
  Problem(int nbVar, const std::vector<std::vector<int>>& clauses,
          const std::vector<int>& projected, const std::vector<int>& protect);

  /**
   * @brief Constructs a deep copy of a problem from a pointer.
   *
   * @param problem Pointer to the existing Problem instance to copy.
   */
  Problem(Problem* problem);

  /**
   * @brief Constructs a deep copy of a problem from a reference.
   *
   * @param problem Reference to the existing Problem instance to copy.
   */
  Problem(Problem& problem);

  /**
   * @brief Destructor. Cleans up the internal vectors.
   */
  ~Problem();

  /**
   * @brief Gets the total number of variables defined in the problem.
   * @return The number of variables.
   */
  inline unsigned getNbVar() { return m_nbVar; }

  /**
   * @brief Gets the set of projected variables.
   * @return A reference to the vector of projected variables.
   */
  inline std::vector<Var>& getProjectedVar() { return m_projected; }

  /**
   * @brief Gets the set of protected variables.
   * @return A reference to the vector of protected variables.
   */
  inline std::vector<Var>& getProtectedVar() { return m_protected; }

  /**
   * @brief Gets the set of clauses forming the CNF formula.
   * @return A reference to the vector of clauses.
   */
  inline std::vector<std::vector<Lit>>& getClauses() { return m_clauses; }

  /**
   * @brief Gets the list of variables prioritized for the solver's decision
   * heap.
   * @return A reference to the vector of heap variables.
   */
  inline std::vector<Var>& getHeapVariable() { return m_heapVariable; }

  /**
   * @brief Updates the total number of variables in the problem.
   * @param nbVar The new number of variables.
   */
  inline void setNbVar(unsigned nbVar) { m_nbVar = nbVar; }

  /**
   * @brief Replaces the current CNF formula with a new set of clauses.
   * @param clauses The new set of clauses to assign.
   */
  inline void setClauses(std::vector<std::vector<Lit>>& clauses) {
    m_clauses = clauses;
  }

  /**
   * @brief Generates a strictly unsatisfiable version of the problem.
   *
   * @return A dynamically allocated Problem pointer containing a trivial
   * contradiction (e.g., x and ¬x).
   */
  Problem* getUnsatProblem();

  /**
   * @brief Simplifies the formula by applying unit propagation.
   *
   * Instantiates a new Problem where the specified unit literals are assumed
   * to be true, removing satisfied clauses and falsified literals from the
   * remaining clauses.
   *
   * @param units The set of unit literals used to condition the formula.
   * @return A dynamically allocated Problem pointer representing the simplified
   * formula.
   */
  Problem* getConditionedFormula(std::vector<Lit>& units);

  /**
   * @brief Prints the entire problem structure (clauses and variables) in
   * DIMACS format.
   *
   * @param[out] out The output stream where the problem is printed.
   */
  void display(std::ostream& out);

  /**
   * @brief Prints statistical information about the problem (e.g., variable
   * count, clause count).
   *
   * @param[in] out The output stream where the statistics are printed.
   * @param[in] startLine A prefix string added to the beginning of each printed
   * line (useful for log parsing).
   */
  void displayStat(std::ostream& out, std::string startLine);
};

}  // namespace bipe