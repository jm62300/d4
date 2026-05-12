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

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "core/SolverTypes.h"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {

/**
 * @brief Concrete implementation of WrapperSolver using the Glucose SAT solver.
 *
 * This class wraps the Glucose solver engine (specifically utilizing
 * `Glucose::Solver`), adapting its internal data structures and methods to
 * satisfy the generic `WrapperSolver` interface used throughout the bipe
 * application.
 */
class WrapperGlucose : public WrapperSolver {
 private:
  Glucose::Solver s;
  Glucose::vec<Glucose::Var> m_setOfVar_m;

  std::vector<Lit> m_assumption;
  std::vector<bool> m_model;
  bool m_activeModel;
  bool m_needModel;

  /**
   * @brief Converts a Glucose internal literal to a standard integer
   * representation.
   *
   * @param l The internal Glucose::Lit.
   * @return A signed integer representing the literal (negative for negated
   * literals).
   */
  inline int litToInt(Glucose::Lit l) {
    return Glucose::sign(l) ? -Glucose::var(l) : Glucose::var(l);
  }

 protected:
  using WrapperSolver::m_isInAssumption;

 public:
  /**
   * @brief Destroys the WrapperGlucose object and cleans up the Glucose engine.
   */
  ~WrapperGlucose() = default;

  /**
   * @brief Initializes the Glucose solver with the clauses from the given
   * problem.
   * @param p The problem containing the CNF formula.
   */
  void initSolver(Problem& p) override;

  /**
   * @brief Executes a resource-limited SAT solving routine using Glucose.
   * @param nbConflict The maximum number of conflicts allowed.
   * @return The status of the search (SAT, UNS, or UND).
   */
  Status solveLimited(int nbConflict) override;

  /**
   * @brief Executes a full, unlimited SAT solving routine using Glucose.
   * @return True if SAT, false if UNSAT.
   */
  bool solve() override;

  /**
   * @brief Enqueues a literal into the Glucose trail.
   * @param l The literal to enqueue.
   */
  void uncheckedEnqueue(Lit l) override;

  /**
   * @brief Checks if a variable has been assigned a truth value in Glucose.
   * @param v The variable to check.
   * @return True if assigned, false otherwise.
   */
  bool varIsAssigned(Var v) override;

  /**
   * @brief Assumes a literal and performs Unit Propagation in Glucose.
   * @param l The literal to assume.
   * @param[out] units The newly deduced unit literals.
   * @return False if a conflict occurs, true otherwise.
   */
  bool decideAndComputeUnit(Lit l, std::vector<Lit>& units) override;

  /**
   * @brief Assumes a set of literals and tests for immediate conflicts.
   * @param lits The list of literals to assume.
   * @param[out] core The extracted unsatisfiable core if a conflict occurs.
   * @return True if successful, false if a conflict occurs.
   */
  bool decideAndTest(std::vector<Lit>& lits, std::vector<Lit>& core) override;

  /**
   * @brief Triggers a solver restart in the Glucose engine.
   */
  void restart() override;

  /**
   * @brief Sets the current assumption stack.
   * @param assums The list of literals to assume.
   */
  void setAssumption(std::vector<Lit>& assums) override;

  /**
   * @brief Retrieves the current assumption stack.
   * @return A reference to the vector of assumed literals.
   */
  std::vector<Lit>& getAssumption() override;

  /**
   * @brief Pushes a literal onto the assumption stack.
   * @param l The literal to assume.
   */
  void pushAssumption(Lit l) override;

  /**
   * @brief Pops one or more literals from the assumption stack.
   * @param count The number of assumptions to remove.
   */
  void popAssumption(unsigned count) override;

  /**
   * @brief Clears the assumption stack.
   */
  void cleanAssumption() override;

  /**
   * @brief Prints the current assumption stack to the given stream.
   * @param out The output stream.
   */
  void displayAssumption(std::ostream& out) override;

  /**
   * @brief Sets whether the solver should construct a model upon a SAT result.
   * @param b True to construct the model.
   */
  void setNeedModel(bool b) override;

  /**
   * @brief Debugging utility that prints the current Glucose assignment trail.
   */
  void showTrail() override;

  /**
   * @brief Retrieves the extracted complete model from Glucose.
   * @return A reference to the boolean vector representing the model.
   */
  std::vector<bool>& getModel() override;

  /**
   * @brief Retrieves the unit clauses deduced at decision level 0.
   * @param[out] units The vector where units will be stored.
   */
  void getUnits(std::vector<Lit>& units) override;

  /**
   * @brief Extracts the unsatisfiable core after an UNSAT result.
   * @param[out] core The vector where the core literals will be stored.
   */
  void getCore(std::vector<Lit>& core) override;

  /**
   * @brief Adds a new permanent clause to the Glucose formula.
   * @param cl The vector of literals representing the clause.
   */
  void addClauseInit(std::vector<Lit>& cl) override;

  /**
   * @brief Sets the index limit for assumptions.
   * @param lastIndex The highest index to process.
   */
  void setLastIndexAssumption(int lastIndex) override;

  /**
   * @brief Temporarily forces Glucose to branch on specific variables next.
   * @param priority The vector of variables to prioritize.
   */
  void oncePriorityVar(std::vector<Var>& priority) override;

  /**
   * @brief Overrides the internal variable decision heap order in Glucose.
   * @param heapVar The new ordered list of variables.
   */
  void setHeapVariable(std::vector<Var>& heapVar) override;

  /**
   * @brief Modifies Glucose's polarity picking heuristic.
   * @param value True to reverse the default polarity.
   */
  void setReversePolarity(bool value) override;

  /**
   * @brief Outputs the current internal formula of Glucose in DIMACS format.
   * @param out The output stream.
   */
  void displayToCnf(std::ostream& out) override;

  /**
   * @brief Configures Glucose for incremental solving.
   * @param m True to enable incremental mode.
   */
  inline void setIncrementalMode(const bool m) override {
    if (m) s.setIncrementalMode();
  }
};

}  // namespace bipe