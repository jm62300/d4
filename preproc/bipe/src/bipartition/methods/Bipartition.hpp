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

#include <functional>
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "src/bipartition/heuristic/HeuristicBipartition.hpp"
#include "src/bipartition/methods/Backbone.hpp"
#include "src/bipartition/methods/Method.hpp"
#include "src/bipartition/option/OptionBipartition.hpp"
#include "src/solver/WrapperSolver.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

namespace bipe {
namespace bipartition {

/**
 * @brief Computes a variable bipartition for a SAT problem.
 *
 * This class implements the bipartition approach presented in:
 * "Definability for model counting" by Jean-Marie Lagniez, Emmanuel Lonca,
 * and Pierre Marquis (Artificial Intelligence 281: 103229, 2020).
 *
 * It partitions the variables of a formula into an input set and an output set,
 * exploiting models, structural gates, and symmetries to minimize the size of
 * the input set required to fully define the remaining variables.
 */
class Bipartition : public Method {
 private:
  const unsigned FREQ_HEADER = 50;
  const unsigned COLUMN_SIZE = 122;
  const unsigned WIDTH_PRINT_COLUMN_MC = 10;

  unsigned m_nbVar = 0;
  unsigned m_nbSATCall = 0;
  unsigned m_nbUNSCall = 0;
  unsigned m_nbUNDCall = 0;
  unsigned m_nbCall = 0;
  unsigned m_nbInputFromModel = 0;
  unsigned m_nbOutputFromSym = 0;
  unsigned m_nbInputFromCompleteModel = 0;
  unsigned m_nbInputFromPadoaModel = 0;

  float m_timeCall = 0.f;
  float m_timeSATCall = 0.f;
  float m_timeUNSCall = 0.f;
  float m_timeUNDCall = 0.f;
  float m_lastCall = 0.f;
  bool m_useCore = false;
  bool m_inputModel = false;
  bool m_useSym = false;

  std::vector<std::vector<int>> m_occurrence;
  std::vector<int> m_nbSatisfied;
  std::vector<int> m_breakcount;
  std::vector<int> m_makecount;
  std::vector<int> m_falsified;
  std::vector<int> m_whereIsFalsified;
  std::vector<Var> m_priority;
  std::vector<Lit> m_breakLit;
  std::vector<bool> m_isInOutput;
  unsigned m_nbFalsified;

  std::vector<bool> m_isProtectedVar;
  std::vector<std::vector<Var>> m_linkedVarTaut;

  std::vector<bool> m_model;
  std::vector<std::vector<bool>> m_listOfModels;
  std::vector<bool> m_marked;

  std::clock_t currentTime;

  /**
   * @brief Initializes the internal timer.
   */
  inline void initTimer() { currentTime = clock(); }

  /**
   * @brief Gets the elapsed time since the timer was initialized.
   * @return The elapsed time in seconds.
   */
  inline float getTimer() {
    return (float)(clock() - currentTime) / CLOCKS_PER_SEC;
  }

  /**
   * @brief Initializes the variables required to execute model exploitation.
   *
   * @param p The problem instance being evaluated.
   */
  void initInputModelVariable(Problem& p);

  /**
   * @brief Prints a horizontal separator line to the log.
   *
   * @param out The output stream used for logging.
   */
  void separator(std::ostream& out);

  /**
   * @brief Prints the table header for the statistics log.
   *
   * @param out The output stream used for logging.
   */
  void printHeader(std::ostream& out);

  /**
   * @brief Prints the current progress and statistics.
   *
   * @param heuristic The heuristic instance used to guide the bipartition.
   * @param out The output stream used for logging.
   */
  void printInfo(HeuristicBipartition* heuristic, std::ostream& out);

  /**
   * @brief Updates internal statistics based on the result of a solver call.
   *
   * @param status The status returned by the last SAT solver call (e.g., SAT,
   * UNSAT, UNDEF).
   */
  void updateInfo(Status status);

  /**
   * @brief Configures the solver's decision priorities based on a specific
   * literal.
   *
   * @param p The problem instance being evaluated.
   * @param solver The underlying SAT solver.
   * @param l The literal to be prioritized during decision making.
   */
  void setPriority(Problem& p, WrapperSolver* solver, Lit l);

  /**
   * @brief Attempts to satisfy the assumptions using previously saved models to
   * avoid SAT calls.
   *
   * @param p The problem instance being evaluated.
   * @param[out] input The current set of identified input variables.
   * @param[in,out] assums The list of assumptions to test against the saved
   * models.
   */
  void tryToConnectSavedModels(Problem& p, std::vector<Var>& input,
                               std::vector<Lit>& assums);

  /**
   * @brief Analyzes a set of known models during preprocessing to quickly
   * identify input variables.
   *
   * @param p The problem instance being evaluated.
   * @param gates The set of structural gates identified in the formula.
   * @param setOfModels A collection of pre-computed complete models.
   * @param[out] collectedInput The set of variables confirmed as inputs.
   */
  void exploitSetOfModelAsPreprocToSpotInput(
      Problem& p, std::vector<Gate>& gates,
      std::vector<std::vector<bool>>& setOfModels,
      std::vector<Var>& collectedInput);

  /**
   * @brief Moves a subset of literals from the assumptions list to the input
   * list.
   *
   * @param[in,out] assums The current list of assumptions. Literals satisfying
   * the test are removed.
   * @param[out] input The list where successfully tested variables are added as
   * inputs.
   * @param test A lambda/functor that returns true if the variable should be
   * moved to the input set.
   * @return The number of elements successfully moved from assumptions to
   * inputs.
   */
  inline unsigned moveToInput(std::vector<Lit>& assums, std::vector<Var>& input,
                              std::function<bool(Var)> test) {
    unsigned j = 0;
    for (unsigned i = 0; i < assums.size(); i++) {
      if (!test(assums[i].var()))
        assums[j++] = assums[i];
      else {
        input.push_back(assums[i].var());
      }
    }

    unsigned ret = assums.size() - j;
    assums.resize(j);
    return ret;
  }

  /**
   * @brief Validates the integrity of the internal break/make count structures
   * (for debugging).
   *
   * @param p The problem instance being evaluated.
   * @param model The complete interpretation currently being considered.
   */
  void debugBreakMakeCount(Problem& p, std::vector<bool>& model);

  /**
   * @brief Flips the truth value of a given variable and updates the make/break
   * structures.
   *
   * @param p The problem instance being evaluated.
   * @param v The variable whose assignment should be flipped.
   * @param[in,out] model The model array to update with the flipped value.
   */
  void flip(Problem& p, Var v, std::vector<bool>& model);

  /**
   * @brief Initializes the make and break count structures from a known
   * satisfiable assignment.
   *
   * @param p The problem instance being evaluated.
   * @param model A known complete model of the problem.
   */
  void initMakeBreakCount(Problem& p, std::vector<bool>& model);

  /**
   * @brief Exploits a complete model to potentially discover input variables
   * for free.
   *
   * @param p The problem instance being evaluated.
   * @param[out] input The list where newly discovered input variables are
   * appended.
   * @param[in,out] assums The remaining assumptions that need testing.
   * @param model The complete model used for analysis.
   * @param lastLit The most recent literal analyzed before calling this
   * function.
   */
  void searchInputFromModel(Problem& p, std::vector<Var>& input,
                            std::vector<Lit>& assums, std::vector<bool>& model,
                            Lit lastLit);

  /**
   * @brief Initializes a Padoa-style definition problem using dual formula
   * copies.
   *
   * Constructs a formula containing two copies of the original problem linked
   * by equivalences. These equivalences can be toggled using selector literals.
   *
   * @param p The original problem.
   * @param[out] res The resulting duplicated problem instance.
   * @param[out] assums The list of assumption literals used to initialize the
   * state.
   * @param[out] mapSelector A map storing the relationship between the added
   * selector variables and the variable pairs they control.
   * @param gates The set of identified structural gates.
   * @param[out] input The set of variables already forced to be in the input
   * set.
   */
  void initProblem(Problem& p, Problem& res, std::vector<Lit>& assums,
                   std::map<Var, std::pair<Var, Var>>& mapSelector,
                   std::vector<Gate>& gates, std::vector<Var>& input);

  /**
   * @brief Generates new output variables by exploiting a provided symmetry
   * group.
   *
   * @param solver The underlying SAT solver containing the problem state.
   * @param mapSelector The map linking selector variables to their controlled
   * equivalence pairs.
   * @param symGroup The defined group of symmetries for the formula.
   * @param def The base set of defining variables to evaluate against the
   * symmetry group.
   * @param areUnit The list of literals currently forced to unit.
   * @param[out] outFound The list where newly deduced output variables are
   * stored.
   */
  void generateOutputSym(WrapperSolver* solver,
                         std::map<Var, std::pair<Var, Var>>& mapSelector,
                         const std::vector<std::vector<Var>>& symGroup,
                         std::vector<Var>& def, std::vector<Lit>& areUnit,
                         std::vector<Var>& outFound);

  /**
   * @brief Executes the core bipartition algorithm using an initialized solver.
   *
   * @param p The initial problem instance.
   * @param solver The solver instance pre-loaded with the dual-formula.
   * @param[out] input The resulting computed partition containing only the
   * input variables.
   * @param[out] und The set of variables whose input/output status remains
   * undetermined.
   * @param heuristic The strategy used to select the next variable to test.
   * @param mapSelector The mapping between selector variables and formula
   * equivalences.
   * @param nbConflict The maximum number of conflicts allowed per SAT solver
   * call.
   * @param symGroup The symmetry groups extracted from the formula.
   * @param out The output stream used for logging progress.
   * @param verbose If true, outputs detailed internal logs about the SAT solver
   * state.
   * @param timer The timer used to strictly enforce execution time limits.
   */
  void compute(Problem& p, WrapperSolver* solver, std::vector<Var>& input,
               std::vector<Lit>& und, HeuristicBipartition* heuristic,
               std::map<Var, std::pair<Var, Var>>& mapSelector,
               const int nbConflict,
               const std::vector<std::vector<Var>>& symGroup, std::ostream& out,
               bool verbose, const Timer& timer);

 public:
  /**
   * @brief Computes a variable bipartition mapping for the given SAT problem.
   *
   * @param p The problem instance to be partitioned.
   * @param[out] input The output container where the final computed input set
   * is stored.
   * @param gates The structural logic gates previously identified in the
   * formula.
   * @param optionBipartition Configuration options detailing limits,
   * heuristics, and strategies.
   * @param symGroup Pre-computed symmetry groups for the formula.
   * @param[in,out] setOfModels A cache of previously discovered complete models
   * to aid preprocessing.
   * @param out The output stream used for logging progress and statistics.
   * @param timer The timer used to strictly enforce execution time limits.
   * @return True if the problem is satisfiable and partitioned, false if it is
   * proven UNSAT.
   */
  bool run(Problem& p, std::vector<Var>& input, std::vector<Gate>& gates,
           const OptionBipartition& optionBipartition,
           const std::vector<std::vector<Var>>& symGroup,
           std::vector<std::vector<bool>>& setOfModels, std::ostream& out,
           const Timer& timer);
};

}  // namespace bipartition
}  // namespace bipe