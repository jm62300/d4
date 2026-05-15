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

#include "src/bipartition/methods/Method.hpp"
#include "src/bipartition/option/Option.hpp"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"

namespace bipe {
namespace bipartition {

/**
 * @brief Extracts a Directed Acyclic Circuit (DAC) from a CNF formula.
 *
 * This class uses Boolean Constraint Propagation (BCP) and structural analysis
 * to identify logical gates (AND, XOR, Equivalence) embedded within a CNF
 * problem. By linking these gates, it constructs a directed acyclic graph
 * (circuit) that represents the functional dependencies between variables,
 * which is highly useful for bipartitioning and variable elimination.
 */
class DACircuit : public Method {
 private:
  const unsigned MAX_SIZE_XOR = 5;

  WrapperSolver* m_solver = nullptr;

  std::vector<bool> m_markedProtected;
  std::vector<bool> m_markedAsOutput;
  std::vector<bool> m_markedAsProjected;
  std::vector<bool> m_marker;
  std::vector<bool> m_markerDescendant;

  std::vector<Var> m_mustUnmark;
  std::vector<Var> m_mustUnmarkDescendant;

  std::vector<std::vector<Var>> m_edgeIn;
  std::vector<std::vector<Var>> m_edgeOut;
  std::vector<std::vector<Lit>> m_impliedList;

  /**
   * @brief Constructs the implication list for every literal via BCP.
   *
   * Uses the internal SAT solver to deduce which literals are forced to be true
   * when a specific literal is assumed true.
   *
   * @param p The problem containing the variables to analyze.
   * @param solver The wrapper for the underlying SAT solver used for
   * propagation.
   * @param[out] impliedList The resulting adjacency list mapping each literal
   * to its implied literals.
   * @param[out] units A collection to store any global unit literals discovered
   * during the process.
   */
  void constructImpliedList(Problem& p, WrapperSolver* solver,
                            std::vector<std::vector<Lit>>& impliedList,
                            std::vector<Lit>& units);

  /**
   * @brief Calculates a heuristic score for a variable based on its implication
   * power.
   *
   * @param v The variable to evaluate.
   * @return A score representing the average number of literals propagated by
   * this variable.
   */
  unsigned getScore(Var v);

  /**
   * @brief Identifies equivalent literals using the previously computed
   * implication lists.
   *
   * @param p The input problem being analyzed.
   * @param[out] listOfGates The collection where newly discovered equivalence
   * gates are stored.
   * @param[out] units A collection to store any global unit literals
   * discovered.
   * @param out The output stream used for logging progress and statistics.
   */
  void identifyEquiv(Problem& p, std::vector<Gate>& listOfGates,
                     std::vector<Lit>& units, std::ostream& out);

  /**
   * @brief Searches the formula for structural or implied AND gates.
   *
   * @param p The problem being analyzed.
   * @param[out] listOfGates The collection where newly discovered AND gates are
   * stored.
   * @param out The output stream used for logging progress.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void identifyAndGate(Problem& p, std::vector<Gate>& listOfGates,
                       std::ostream& out, const Timer& timer);

  /**
   * @brief Searches the formula for structural XOR gates up to a maximum size.
   *
   * @param p The problem being analyzed.
   * @param[out] listOfGates The collection where newly discovered XOR gates are
   * stored.
   * @param out The output stream used for logging progress.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void identifyXorGate(Problem& p, std::vector<Gate>& listOfGates,
                       std::ostream& out, const Timer& timer);

  /**
   * @brief Initializes internal data structures and states before running the
   * DAC extraction.
   *
   * @param p The problem instance being analyzed.
   * @param[out] units The collection to store initially detected unit literals.
   */
  void init(Problem& p, std::vector<Lit>& units);

  /**
   * @brief Creates directed edges in the internal graph representation.
   *
   * Links input variables to their resulting output variables to form the
   * circuit hierarchy.
   *
   * @param in The variables that act as inputs to the gate.
   * @param out The variables that act as outputs of the gate.
   */
  void addRelation(std::vector<Var>& in, std::vector<Var>& out);

  /**
   * @brief Analyzes a subset of clauses to determine if they collectively form
   * an XOR constraint.
   *
   * @param clauses The full set of clauses in the formula.
   * @param indices The specific indices of the clauses suspected of forming an
   * XOR gate.
   * @param[out] xorClause The resulting XOR constraint if successfully
   * detected.
   * @return True if the clauses form a valid XOR constraint, false otherwise.
   */
  bool canWeBuildXor(std::vector<std::vector<Lit>>& clauses,
                     std::vector<int>& indices, std::vector<Lit>& xorClause);

  /**
   * @brief Recursively marks all descendants of a variable in the directed
   * graph.
   *
   * Used for cycle detection and ensuring the circuit remains acyclic.
   *
   * @param v The root variable from which to start marking descendants.
   */
  void markDescendant(Var v);

  /**
   * @brief Clears all descendant markers set during the `markDescendant`
   * operation.
   */
  void unmarkDescendant();

 public:
  /**
   * @brief Constructs a list of disjoint equivalence classes.
   *
   * Groups variables that strictly imply each other into equivalence classes
   * to simplify the formula structure.
   *
   * @param p The problem containing the variables.
   * @param gates The set of initially detected equivalence gates.
   * @param[out] equivClassList The resulting collection of disjoint equivalence
   * classes.
   */
  void extractEquivClass(Problem& p, std::vector<Gate>& gates,
                         std::vector<std::vector<Lit>>& equivClassList);

  /**
   * @brief Executes the full Directed Acyclic Circuit (DAC) extraction process.
   *
   * Orchestrates the search for equivalences, AND gates, and XOR gates,
   * building a directed graph of functional dependencies while maintaining an
   * acyclic property.
   *
   * @param p The problem instance from which to extract the DAC.
   * @param[out] listOfGates The resulting collection of ordered gates that form
   * the circuit.
   * @param[out] setOfModels A collection used to store any valid models found
   * during internal BCP.
   * @param optDac Configuration options controlling gate extraction limits and
   * strategies.
   * @param out The output stream used for logging progress and statistics.
   * @param timer The timer used to strictly enforce execution time limits.
   * @return True if the problem remains satisfiable after extraction, false if
   * proven UNSAT.
   */
  bool run(Problem& p, std::vector<Gate>& listOfGates,
           std::vector<std::vector<bool>>& setOfModels, const OptionDac& optDac,
           std::ostream& out, const Timer& timer);
};

}  // namespace bipartition
}  // namespace bipe