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

#include <ostream>
#include <vector>

#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {

/**
 * @brief Represents the possible statuses returned by a SAT solver.
 */
enum Status { UNS, SAT, UND };

/**
 * @brief Abstract base class providing a unified interface for SAT solvers.
 *
 * This wrapper allows the core algorithms (like bipartitioning or counting)
 * to remain agnostic to the specific underlying SAT solver backend (e.g.,
 * Glucose, MiniSat). It defines standard operations for incremental solving,
 * assumption management, and extraction of cores/models.
 */
class WrapperSolver {
 protected:
  /// Tracks which variables are currently in the assumption stack.
  std::vector<char> m_isInAssumption;

 public:
  /**
   * @brief Factory method to instantiate a specific SAT solver wrapper.
   *
   * @param solverName The string identifier of the solver to load (e.g.,
   * "glucose").
   * @param out The output stream used for logging solver initialization.
   * @return A pointer to the dynamically allocated WrapperSolver instance.
   */
  static WrapperSolver* makeWrapperSolver(const std::string solverName,
                                          std::ostream& out);

  /**
   * @brief Virtual destructor to ensure proper cleanup of the underlying
   * solver.
   */
  virtual ~WrapperSolver() {}

  /**
   * @brief Initializes the underlying solver with the clauses from a problem.
   * @param p The problem containing the CNF formula to load.
   */
  virtual void initSolver(Problem& p) = 0;

  /**
   * @brief Executes a resource-limited SAT solving routine.
   * @param nbConflict The maximum number of conflicts allowed before aborting.
   * @return The status of the search: SAT, UNS (Unsatisfiable), or UND
   * (Undetermined/Aborted).
   */
  virtual Status solveLimited(int nbConflict) = 0;

  /**
   * @brief Executes a full SAT solving routine without limits.
   * @return True if the formula is satisfiable, false if unsatisfiable.
   */
  virtual bool solve() = 0;

  /**
   * @brief Enqueues a literal into the solver's trail without immediately
   * checking for conflicts.
   * @param l The literal to enqueue.
   */
  virtual void uncheckedEnqueue(Lit l) = 0;

  /**
   * @brief Triggers a solver restart, clearing the current decision trail.
   */
  virtual void restart() = 0;

  /**
   * @brief Replaces the current assumption stack with a new set of assumptions.
   * @param assums The list of literals to assume.
   */
  virtual void setAssumption(std::vector<Lit>& assums) = 0;

  /**
   * @brief Retrieves the current assumption stack.
   * @return A reference to the vector of assumed literals.
   */
  virtual std::vector<Lit>& getAssumption() = 0;

  /**
   * @brief Pushes a single literal onto the assumption stack.
   * @param l The literal to assume.
   */
  virtual void pushAssumption(Lit l) = 0;

  /**
   * @brief Pops one or more literals from the top of the assumption stack.
   * @param count The number of assumptions to remove (default is 1).
   */
  virtual void popAssumption(unsigned count = 1) = 0;

  /**
   * @brief Completely clears the assumption stack.
   */
  virtual void cleanAssumption() = 0;

  /**
   * @brief Prints the current assumption stack to the provided stream.
   * @param out The output stream.
   */
  virtual void displayAssumption(std::ostream& out) = 0;

  /**
   * @brief Checks if a variable has been assigned a truth value in the current
   * trail.
   * @param v The variable to check.
   * @return True if the variable is assigned, false if it is unassigned.
   */
  virtual bool varIsAssigned(Var v) = 0;

  /**
   * @brief Flags whether the solver should construct and store a full model
   * upon finding a SAT result.
   * @param b True to construct a model, false to save memory/time if only the
   * SAT status is needed.
   */
  virtual void setNeedModel(bool b) = 0;

  /**
   * @brief Debugging utility that prints the current state of the assignment
   * trail.
   */
  virtual void showTrail() = 0;

  /**
   * @brief Retrieves the complete model found during the last successful solve.
   * @return A reference to the boolean vector representing the truth values of
   * all variables.
   */
  virtual std::vector<bool>& getModel() = 0;

  /**
   * @brief Retrieves the set of unit clauses deduced at decision level 0.
   * @param[out] units The vector where the extracted unit literals will be
   * stored.
   */
  virtual void getUnits(std::vector<Lit>& units) = 0;

  /**
   * @brief Modifies the solver's default phase saving / polarity picking
   * heuristic.
   * @param value True to reverse the default polarity.
   */
  virtual void setReversePolarity(bool value) = 0;

  /**
   * @brief Outputs the current internal state of the formula in DIMACS CNF
   * format.
   * @param out The output stream.
   */
  virtual void displayToCnf(std::ostream& out) = 0;

  /**
   * @brief Configures the solver for incremental usage (keeping internal states
   * intact between calls).
   * @param m True to enable incremental mode.
   */
  virtual void setIncrementalMode(const bool m) = 0;

  /**
   * @brief Extracts the unsatisfiable core (a conflicting subset of
   * assumptions) after an UNSAT result.
   * @param[out] core The vector where the core literals will be stored.
   */
  virtual void getCore(std::vector<Lit>& core) = 0;

  /**
   * @brief Adds a new permanent clause to the solver's initial formula.
   * @param cl The vector of literals representing the clause.
   */
  virtual void addClauseInit(std::vector<Lit>& cl) = 0;

  /**
   * @brief Sets the index limit for assumptions to be considered in the current
   * solver call.
   * @param lastIndex The highest index in the assumption array to process.
   */
  virtual void setLastIndexAssumption(int lastIndex) = 0;

  /**
   * @brief Temporarily forces the solver to branch on specific variables next.
   * @param priority The vector of variables to prioritize.
   */
  virtual void oncePriorityVar(std::vector<Var>& priority) = 0;

  /**
   * @brief Completely overrides the solver's internal variable decision
   * heap/order.
   * @param heapVar The new ordered list of variables.
   */
  virtual void setHeapVariable(std::vector<Var>& heapVar) = 0;

  /**
   * @brief Assumes a literal and performs Unit Propagation to deduce forced
   * assignments.
   * @param l The literal to assume.
   * @param[out] units The newly deduced unit literals.
   * @return False if the propagation immediately results in a conflict, true
   * otherwise.
   */
  virtual bool decideAndComputeUnit(Lit l, std::vector<Lit>& units) = 0;

  /**
   * @brief Assumes a set of literals and checks for immediate conflicts.
   * @param lits The list of literals to assume.
   * @param[out] core The extracted unsatisfiable core if a conflict is found.
   * @return True if propagation succeeds without conflict, false if a conflict
   * occurs.
   */
  virtual bool decideAndTest(std::vector<Lit>& lits,
                             std::vector<Lit>& core) = 0;

  /**
   * @brief Gets the current number of literals in the assumption stack.
   * @return The size of the assumption stack.
   */
  unsigned sizeAssumption() { return getAssumption().size(); }

  /**
   * @brief Checks if a variable is currently part of the active assumption
   * list.
   *
   * @param v The variable to check.
   * @return True if 'v' is in the assumption list, false otherwise.
   */
  inline bool isInAssumption(Var v) { return m_isInAssumption[v]; }

  /**
   * @brief Clears the entire assumption stack rapidly.
   */
  inline void resetAssumption() { popAssumption(getAssumption().size()); }
};

}  // namespace bipe