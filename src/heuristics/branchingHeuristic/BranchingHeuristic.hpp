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

#include "src/heuristics/phaseSelection/PhaseHeuristic.hpp"
#include "src/heuristics/scoringVariable/ScoringMethod.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"

namespace d4 {
const unsigned SIZE_PAGE_LIST_LIT = 1 << 10;

class ListLit {
 private:
  static unsigned s_posPage;
  static Lit *s_currentPage;
  static std::vector<Lit *> s_pages;
  static std::vector<std::vector<Lit *>> s_availaible;

 private:
  int m_size;
  Lit *m_array;

 public:
  ListLit();
  ListLit(const Lit *tab, int size);
  virtual ~ListLit();

  void setListLit(const Lit *tab, int size);

  inline unsigned size() { return m_size; }
  inline void setSize(int size) { m_size = size; };
  inline void setArray(Lit *array) { m_array = array; }
  inline Lit &operator[](int index) {
    assert(index < m_size);
    return m_array[index];
  }
};

class BranchingHeuristic {
 protected:
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  FormulaManager *m_specs;
  ProblemManager *m_problem;
  std::vector<bool> m_isDecisionVariable;
  unsigned m_freqDecay;
  unsigned m_nbCall;

 public:
  /**
   * @brief Remove the defaut constructor.
   *
   */
  BranchingHeuristic() = delete;

  /**
   * @brief Construct a new Branching Heuristic object.
   *
   * @param options are the options.
   * @param problem gives the problem we are considering.
   * @param specs gives the real time information about the formula.
   * @param solver the solver (used for VSADS/VSIDS)
   * @param out is the stream where are printed out the information.
   */
  BranchingHeuristic(const OptionBranchingHeuristic &options,
                     ProblemManager *problem, FormulaManager *specs,
                     WrapperSolver *solver, std::ostream &out);

  /**
   * @brief Destroy the Branching Heuristic object.
   */
  virtual ~BranchingHeuristic();

  /**
   * @brief Factory called for constructing a branching heuristic.
   *
   * @param options gives the options.
   * @param problem gives the problem we are considering.
   * @param specs gives the real time information about the formula.
   * @param solver the solver (used for VSADS/VSIDS)
   * @param out is the stream where are printed out the information.
   *
   * @return a branching heuristic that fits the options.
   */
  static BranchingHeuristic *makeBranchingHeuristic(
      const OptionBranchingHeuristic &options, ProblemManager *problem,
      FormulaManager *specs, WrapperSolver *solver, std::ostream &out);

  /**
   * @brief Select a list of literals we want to branch on it in a deterministic
   * way.
   *
   * @param vars is the set of variables under consideration.
   * @param[out] lits is the place where are stored the literals we are
   * considering.
   */
  virtual void selectLitSet(std::vector<Var> &vars, ListLit &lits) = 0;
};
}  // namespace d4