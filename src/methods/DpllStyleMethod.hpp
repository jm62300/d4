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

#include <ctime>
#include <iomanip>
#include <iostream>
#include <span>

#include "Counter.hpp"
#include "MethodManager.hpp"
#include "SemiringConcept.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/heuristics/branchingHeuristic/BranchingHeuristic.hpp"
#include "src/heuristics/partialOrder/PartialOrderHeuristic.hpp"
#include "src/heuristics/partialOrder/PartialOrderHeuristicNone.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/utils/MemoryStat.hpp"

#define NB_SEP_MC 92
#define MASK_SHOWRUN_MC ((2 << 16) - 1)
#define WIDTH_PRINT_COLUMN_MC 12
#define MASK_HEADER 33554431

namespace d4 {
template <class T>
class Counter;

template <NumberType T, SemiringPolicy<T> O>
class DpllStyleMethod : public MethodManager, public Counter<T> {
 private:
  bool optDomConst;
  bool optReversePolarity;
  bool m_verbosity;

  unsigned m_nbCallCall;
  unsigned m_nbSplit;
  unsigned m_nbDecisionNode;
  unsigned m_optCached;
  unsigned m_stampIdx;
  unsigned m_freqDecay;
  bool m_isProjectedMode;

  bool m_connectedComponent;
  unsigned m_lastNbSplit;
  unsigned m_nbFailedIncreased;
  unsigned m_limitNbFailedInRaw = 11;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> m_clauses;
  std::vector<bool> m_isDecisionVariable;

  std::vector<uint8_t> m_signLit;

  WrapperSolver* m_solver;
  FormulaManager* m_specs;

  BranchingHeuristic* m_heuristic;
  TmpEntry<T> NULL_CACHE_ENTRY;
  CacheManager<T>* m_cache;

  std::ostream& m_out;
  O m_semiringOps;

 public:
  /**
   * @brief Constructs the DPLL-style solver method.
   *
   * @param[in] options     The configuration options for the DPLL method.
   * @param[in] initProblem Pointer to the problem manager instance.
   * @param[in] out         The output stream to be used for logging or results.
   */
  DpllStyleMethod(const OptionDpllStyleMethod& options,
                  const ProblemManager& problem, std::ostream& out)
      : m_out(out) {
    m_out << "c [DPLL STYLE METHOD]" << options << "\n";
    m_verbosity = options.verbosity;

    // we create and init the solver.
    m_solver =
        WrapperSolver::makeWrapperSolver(options.optionSolver, problem, m_out);
    m_solver->initSolver(problem);
    m_solver->setNeedModel(true);

    assert(problem.getQuantification().size() == 1);
    m_semiringOps = O(problem.getNbVar(), problem.getWeightMap());
    m_isProjectedMode = problem.getQuantification()[0].size() > 0;
    m_connectedComponent = true;
    m_nbFailedIncreased = m_lastNbSplit = 0;

    // we initialize the object that will give info about the problem.
    m_specs = FormulaManager::makeFormulaManager(options.optionSpecManager,
                                                 problem, m_out);

    // we initialize the object used to compute score and partition.
    m_heuristic = BranchingHeuristic::makeBranchingHeuristic(
        options.optionBranchingHeuristic, problem, m_specs, *m_solver,
        *m_solver, m_out);

    // specify which variables are decisions, and which are not.
    m_isDecisionVariable.clear();
    m_isDecisionVariable.resize(problem.getNbVar() + 1, !m_isProjectedMode);
    for (auto v : problem.getQuantification()[0])
      m_isDecisionVariable[v] = true;

    m_cache = CacheManager<T>::makeCacheManager(
        options.optionCacheManager, problem.getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    m_optCached = options.optionCacheManager.isActivated;
    m_nbDecisionNode = m_nbSplit = m_nbCallCall = 0;
    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);

    m_signLit.resize(1 + problem.getNbVar(), 0);
  }  // constructor

  /**
     Destructor.
   */
  ~DpllStyleMethod() {
    delete m_solver;
    delete m_specs;
    delete m_heuristic;
    delete m_cache;
  }  // destructor

  inline O getSemiring() { return m_semiringOps; }

 private:
  /**
     Expel from a set of variables the ones they are marked as being decidable.

     @param[out] vars, the set of variables we search to filter.

     @param[in] isDecisionvariable, a boolean vector that marks as true decision
     variables.
   */
  void expelNoDecisionVar(std::vector<Var>& vars,
                          std::vector<bool>& isDecisionVariable) {
    if (!m_isProjectedMode) return;

    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++)
      if (isDecisionVariable[vars[i]]) vars[j++] = vars[i];
    vars.resize(j);
  }  // expelNoDecisionVar

  /**
     Expel from a set of variables the ones they are marked as being decidable.

     @param[out] lits, the set of literals we search to filter.

     @param[in] isDecisionvariable, a boolean vector that marks as true decision
     variables.
   */
  void expelNoDecisionLit(std::vector<Lit>& lits,
                          std::vector<bool>& isDecisionVariable) {
    if (!m_isProjectedMode) return;

    unsigned j = 0;
    for (unsigned i = 0; i < lits.size(); i++)
      if (isDecisionVariable[lits[i].var()]) lits[j++] = lits[i];
    lits.resize(j);
  }  // expelNoDecisionLit

  /**
     Compute the current priority set.

     @param[in] connected, the current component.
     @param[in] priorityVar, the current priority variables.
     @param[out] currPriority, the intersection of the two previous sets.
  */
  inline void computePrioritySubSet(std::vector<Var>& connected,
                                    std::vector<Var>& priorityVar,
                                    std::vector<Var>& currPriority) {
    currPriority.resize(0);
    m_stampIdx++;
    for (auto& v : connected) m_stampVar[v] = m_stampIdx;
    for (auto& v : priorityVar)
      if (m_stampVar[v] == m_stampIdx && !m_specs->varIsAssigned(v))
        currPriority.push_back(v);
  }  // computePrioritySet

  /**
     Print out information about the solving process.

     @param[in] out, the stream we use to print out information.
  */
  inline void showInter(std::ostream& out) {
    if (!m_verbosity) return;
    out << "c " << std::fixed << std::setprecision(2) << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << getTimer() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->getNbPositiveHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cache->getNbNegativeHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->usedMemory() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplit << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbDecisionNode << "|\n";
  }  // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream& out) {
    out << "c ";
    for (int i = 0; i < NB_SEP_MC; i++) out << "-";
    out << "\n";
  }  // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream& out) {
    if (!m_verbosity) return;
    separator(out);
    out << "c " << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node" << "|\n";
    separator(out);
  }  // showHeader

  /**
     Print out information when it is requiered.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream& out) {
    if (!(m_nbCallCall & (MASK_HEADER))) showHeader(out);
    if (m_nbCallCall && !(m_nbCallCall & MASK_SHOWRUN_MC)) {
      showInter(out);
    }
  }  // showRun

  /**
     Print out the final stat.

     @param[in] out, the stream we use to print out information.
   */
  inline void printFinalStats(std::ostream& out) {
    separator(out);
    out << "c\n";
    out << "c \033[1m\033[31mStatistics \033[0m\n";
    out << "c \033[33mCompilation Information\033[0m\n";
    out << "c Number of recursive call: " << m_nbCallCall << "\n";
    out << "c Number of split formula: " << m_nbSplit << "\n";
    out << "c Number of decision: " << m_nbDecisionNode << "\n";
    out << "c\n";
    m_specs->printInformation(out);
    m_cache->printCacheInformation(out);
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  }  // printFinalStat

  /**
     Initialize the assumption in order to compute compiled formula under this
     one.

     @param[in] assums, the assumption
  */
  inline void initAssumption(const std::vector<Lit>& assums) {
    m_solver->restart();
    m_solver->popAssumption(m_solver->getAssumption().size());
    m_solver->setAssumption(assums);
  }  // initAssumption

  /**
     Decide if the cache is realized or not.
   */
  bool cacheIsActivated(std::span<Var>& connected) {
    if (!m_optCached) return false;
    return m_cache->isActivated(connected.size());
  }  // cacheIsActivated

  /**
   * @brief Compute the connected component.
   *
   * @param setOfVar is the set of variables under consideration.
   * @param varConnected are the computed connected component.
   * @param freeVariable are the free variables.
   * @return is the number of components.
   */
  inline int computeConnectedComponent(
      std::span<Var> setOfVar, std::vector<std::vector<Var>>& varConnected,
      std::vector<Var>& freeVariable) {
    if (m_connectedComponent && !(m_nbCallCall % 100000)) {
      if (m_lastNbSplit == m_nbSplit)
        m_nbFailedIncreased++;
      else {
        m_nbFailedIncreased = 0;
        m_lastNbSplit = m_nbSplit;
      }

      m_connectedComponent = m_nbFailedIncreased < m_limitNbFailedInRaw;
      if (!m_connectedComponent)
        std::cout << "c [CONNECTED COMPONENT] Stop searching for connected "
                     "component\n";
    }

    if (m_connectedComponent || !(m_nbCallCall % 500)) {
      unsigned ret = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                        freeVariable);

      if (ret > 1 && !m_connectedComponent) {
        std::cout << "c [CONNECTECT COMPONENT] Start for searching for "
                     "connected component\n";
        m_nbFailedIncreased = 0;
        m_limitNbFailedInRaw++;
        m_connectedComponent = true;
      }
      return ret;
    }

    // move the free variables.
    return m_specs->computeTrivialConnectedComponent(varConnected, setOfVar,
                                                     freeVariable);
  }  // computeConnectedComponent

  /**
   * Compile the CNF formula into a FBDD.
   *
   * @param[in] setOfVar, the current set of considered variables
   * @param[in] unitsLit, the set of unit literal detected at this level
   * @param[in] freeVariable, the variables which become free
   * @param[in] out, the stream we use to print out information.
   *
   * \return an element of type U that sums up the given CNF sub-formula
   * using a DPLL style algorithm with an operation manager.
   */
  T compute_(std::span<Var> setOfVar, std::vector<Lit>& unitsLit,
             std::vector<Var>& freeVariable, std::ostream& out) {
    showRun(out);
    m_nbCallCall++;
    /// if (m_nbCallCall > 10000000) exit(0);

    if (!m_solver->solve(setOfVar, unitsLit)) return m_semiringOps.zero();
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    const int nbComponent =
        computeConnectedComponent(setOfVar, varConnected, freeVariable);
    expelNoDecisionVar(freeVariable, m_isDecisionVariable);

    // consider each connected component.
    if (nbComponent) {
      m_nbSplit += (nbComponent > 1) ? nbComponent : 0;
      T ret = m_semiringOps.presetMul(nbComponent);
      for (int cp = 0; cp < nbComponent; cp++) {
        std::span<Var> connected(varConnected[cp].begin(),
                                 varConnected[cp].size());
        // std::vector<Var>& connected = varConnected[cp];

        bool cacheActivated = cacheIsActivated(connected);
        TmpEntry<T> cb = cacheActivated ? m_cache->searchInCache(connected)
                                        : NULL_CACHE_ENTRY;
        if (cacheActivated && cb.defined) {
          m_semiringOps.mul(ret, cb.getValue());
        } else {
          // recursive call
          T tmp = computeDecisionNode(connected, out);
          if (cacheActivated) m_cache->addInCache(cb, tmp);
          m_semiringOps.mul(ret, tmp);
        }
      }

      m_specs->postUpdate(unitsLit);
      expelNoDecisionLit(unitsLit, m_isDecisionVariable);

      return ret;
    }

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit, m_isDecisionVariable);
    return m_semiringOps.one();
  }  // compute_

  /**
     This function select a variable and compile a decision node.

     @param[in] connected, the set of variable present in the current
     problem.
     @param[in] out, the stream whare are printed out the logs.

     \return the compiled formula.
  */
  T computeDecisionNode(std::span<Var> connected, std::ostream& out) {
    // search the next variable to branch on
    ListLit lits;
    m_heuristic->selectLitSet(connected, lits);
    if (!lits.size()) return m_semiringOps.one();
    m_nbDecisionNode++;

    // compile the formula where l is assigned to true
    std::vector<Lit> units;
    std::vector<Var> free;
    T ret = m_semiringOps.presetSum(lits.size() + 1);
    unsigned nb = 0, sizeAssum = m_solver->sizeAssumption();
    for (unsigned i = 0; i <= lits.size(); i++) {
      if (i != 0) {
        m_solver->popAssumption();
        m_solver->pushAssumption(~lits[i - 1]);
        if (lits.size() > 1 && !m_solver->solve(connected, units)) break;
      }

      if (i != lits.size()) m_solver->pushAssumption(lits[i]);
      units.clear();
      free.clear();
      T tmp = compute_(connected, units, free, out);
      m_semiringOps.add(ret, tmp, units, free);
    }

    // reinit some variables.
    assert(m_solver->sizeAssumption() > sizeAssum);
    m_solver->popAssumption(m_solver->sizeAssumption() - sizeAssum);

    return ret;
  }  // computeDecisionNode

  /**
     Compute U using the trace of a SAT solver.

     @param[in] setOfVar, the set of variables of the considered problem.
     @param[in] out, the stream are is print out the logs.
     @param[in] warmStart, to activate/deactivate the warm start strategy.
     /!\ When the warm start is activated the assumptions are reset.

     \return an element of type U that sums up the given CNF formula using a
     DPLL style algorithm with an operation manager.
  */
  T compute(std::vector<Var>& setOfVar, std::ostream& out,
            bool warmStart = true) {
    std::vector<Var> decisionVar;
    for (auto& v : setOfVar)
      if (m_isDecisionVariable[v]) decisionVar.push_back(v);

    if (warmStart && !m_solver->warmStart(29, 11, decisionVar, m_out))
      return m_semiringOps.zero();

    std::vector<Lit> units;
    std::vector<Var> free;
    T ret = m_semiringOps.presetSum(1);
    T tmp = compute_(setOfVar, units, free, out);
    return m_semiringOps.add(ret, tmp, units, free);
  }  // compute

 public:
  /**
     Given an assumption, we compute the number of models.  That is
     different from the query strategy, where we first compute and then
     condition the computed structure.

     @param[in] setOfVar, the set of variables of the considered problem.
     @param[in] assumption, the set of literals we want to assign.
     @param[in] out, the stream where are print out the log.

     \return the number of models when the formula is simplified by the
     given assumption.
   */
  T count(std::vector<Var>& setOfVar, std::vector<Lit>& assumption,
          std::ostream& out) {
    initAssumption(assumption);

    // get the unit not in setOfVar.
    std::vector<Lit> shadowUnits;
    m_stampIdx++;
    for (auto& v : setOfVar) m_stampVar[v] = m_stampIdx;
    for (auto& l : assumption)
      if (m_stampVar[l.var()] != m_stampIdx) shadowUnits.push_back(l);

    m_specs->preUpdate(shadowUnits);
    T result = compute(setOfVar, out, false);
    m_specs->postUpdate(shadowUnits);

    return result;
  }  // count

  /**
     Run the DPLL style algorithm with the operation manager.

     @param[in] vm, the set of options.
   */
  T run() {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);

    T result = compute(setOfVar, m_out);
    printFinalStats(m_out);
    return result;
  }  // run
};

}  // namespace d4
