/**
 * reducer
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

#include "src/utils/ProblemTypes.hpp"
namespace bipe {
namespace reducer {

/**
 * @brief Represents an implication list for binary clauses.
 *
 * Uses the struct hack (zero-length array) to dynamically allocate
 * adjacent memory for the literals implied by a specific literal.
 */
struct Imply {
  unsigned size;
  Lit lits[0];
  Lit& operator[](std::size_t idx) { return lits[idx]; }
};

typedef unsigned CRef;  // Clause Reference ID

/**
 * @brief Represents a watch list for non-binary clauses.
 *
 * Stores the references (CRef) of clauses watching a specific literal.
 */
struct Watch {
  unsigned size;
  CRef watches[0];

  inline void push(CRef cref) { watches[size++] = cref; }
};

/**
 * @brief The Boolean Constraint Propagation (BCP) engine.
 *
 * Manages clause assignments, the literal trail, and the 2-watched-literal
 * scheme to efficiently deduce implied literals and detect conflicts.
 */
class Propagator {
 private:
  std::ostream& m_out;

  uint8_t* m_data;

  unsigned m_nbVar;
  unsigned m_posClauseNotBin;
  bool m_isUnsat;

  std::vector<CRef> m_notBinClauseRefs;
  std::vector<Imply*> m_binListRefs;
  std::vector<Watch*> m_watchList;

  Lit* m_trail;
  unsigned m_trailSize;
  unsigned m_trailLimUnit;
  unsigned m_trailPos;

  lbool* m_assign;
  bool m_verbose;

  /**
   * @brief Generates a trivially unsatisfiable formula.
   *
   * Replaces the current problem with a minimal conflicting set of clauses.
   *
   * @param[out] clauses The vector where the UNSAT problem will be stored.
   */
  inline void generateUnsat(std::vector<std::vector<Lit>>& clauses) {
    clauses.clear();
    clauses.push_back({Lit::makeLitTrue(1)});
    clauses.push_back({Lit::makeLitFalse(1)});
  }

  /**
   * @brief Deletes the default constructor to enforce proper initialization.
   */
  Propagator() = delete;

 public:
  /// Gets the list of clause references for all non-binary clauses.
  inline std::vector<CRef>& getNotBinClauses() { return m_notBinClauseRefs; }

  /// Retrieves a clause object from memory using its reference ID.
  inline Clause& getClause(CRef cref) { return *((Clause*)&m_data[cref]); }

  /// Sets the unsat status of the propagator.
  inline void setIsUnsat(bool val) { m_isUnsat = val; }

  /// Checks if the current formula is proven unsatisfiable.
  inline bool getIsUnsat() { return m_isUnsat; }

  /// Gets the total number of variables managed by the propagator.
  inline unsigned getNbVar() { return m_nbVar; }

  /// Retrieves the list of literals implied by a given literal.
  inline Imply* litImplied(Lit l) { return m_binListRefs[l.intern()]; }

  /// Gets the current size of the propagation trail.
  inline unsigned getTrailSize() { return m_trailSize; }

  /**
   * @brief Constructs a new Propagator object.
   *
   * @param nbVar The number of variables in the problem.
   * @param clauses The set of initial clauses to load into the propagator.
   * @param out The output stream used for logging.
   * @param verbose If true, prints detailed progress information.
   */
  Propagator(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
             std::ostream& out, bool verbose = false);

  /**
   * @brief Constructs a new Propagator using standard output for logging.
   *
   * @param nbVar The number of variables in the problem.
   * @param clauses The set of initial clauses to load into the propagator.
   * @param verbose If true, prints detailed progress information.
   */
  Propagator(unsigned nbVar, std::vector<std::vector<Lit>>& clauses,
             bool verbose = false)
      : Propagator(nbVar, clauses, std::cout, verbose) {}

  /**
   * @brief Destroys the Propagator object and frees internal memory.
   */
  ~Propagator();

  /**
   * @brief Adds a binary clause to the propagator.
   *
   * @param a The first literal of the clause.
   * @param b The second literal of the clause.
   */
  void addBinary(Lit a, Lit b);

  /**
   * @brief Adds a general (non-binary) clause to the propagator.
   *
   * @param clause The vector of literals representing the clause.
   */
  void addClause(std::vector<Lit>& clause);

  /**
   * @brief Gets the current truth value of a specific literal.
   *
   * @param l The literal to evaluate.
   * @return An lbool representing the truth value (e.g., l_True, l_False, or
   * l_Undef).
   */
  inline lbool value(const Lit l) {
    return ((uint8_t)l.sign()) ^ m_assign[l.var()];
  }

  /**
   * @brief Enqueues a literal into the propagation trail without checking for
   * conflicts.
   *
   * @param l The literal to push onto the trail.
   */
  void uncheckedEnqueue(Lit l);

  /**
   * @brief Executes the unit propagation (BCP) process.
   *
   * @return true if propagation succeeds, false if a conflict occurs.
   */
  bool propagate();

  /**
   * @brief Propagates literals currently in the trail at decision level zero.
   *
   * Sets the UNSAT flag if a conflict is found at level zero.
   */
  void propagateLevelZero() {
    m_isUnsat = !propagate();
    m_trailLimUnit = m_trailSize;
  }

  /**
   * @brief Outputs the current problem in DIMACS CNF format.
   *
   * @param out The output stream to print the formatted problem to.
   */
  void display(std::ostream& out);

  /**
   * @brief Attaches a clause to the literal watch lists.
   *
   * @param cref The reference ID of the clause to attach.
   */
  void attachClause(CRef cref);

  /**
   * @brief Detaches a clause from the literal watch lists.
   *
   * @param cref The reference ID of the clause to detach.
   */
  void detachClause(CRef cref);

  /**
   * @brief Extracts the current state of the clauses.
   *
   * @param[out] clauses The vector where the extracted clauses will be stored.
   */
  void extractFormula(std::vector<std::vector<Lit>>& clauses);

  /**
   * @brief Restarts the propagation state, clearing the assignment trail.
   */
  void restart();

  /**
   * @brief Backtracks the propagation trail to a specific position.
   *
   * Undoes literal assignments until the trail size matches the given position.
   *
   * @param pos The target size of the trail after backtracking.
   */
  void cancelUntilPos(unsigned pos);
};

}  // namespace reducer
}  // namespace bipe