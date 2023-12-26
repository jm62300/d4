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

#include <gmpxx.h>

#include <ctime>
#include <iomanip>
#include <iostream>

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/heuristics/BranchingHeuristic.hpp"
#include "src/heuristics/partitioning/PartitioningHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/methods/OptionQbfCounter.hpp"
#include "src/options/solvers/OptionSolver.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/qbf/ProblemManagerQbf.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/utils/MemoryStat.hpp"

#define NB_SEP_QBF_MC 92
#define MASK_SHOWRUN_MC ((2 << 13) - 1)
#define WIDTH_PRINT_COLUMN_MC 12
#define MASK_HEADER 1048575

#include "CountingOperation.hpp"
#include "DecisionDNNFOperation.hpp"
#include "OperationManager.hpp"

namespace d4 {
template <class T>
class Counter;

class QbfCounter : public MethodManager {
 private:
  bool optDomConst;
  bool optReversePolarity;

  unsigned m_nbCallCall;
  unsigned m_nbSplit;
  unsigned m_nbDecisionNode;
  unsigned m_optCached;
  unsigned m_stampIdx;
  unsigned m_freqDecay;
  bool m_isProjectedMode;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> m_clauses;
  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_currentPrioritySet;
  std::vector<unsigned> m_varBlockLevel;

  ProblemManagerQbf *m_problem;
  WrapperSolver *m_solver;
  SpecManager *m_specs;

  BranchingHeuristic *m_heuristic;
  TmpEntry<mpz::mpz_int> NULL_CACHE_ENTRY;
  CacheManager<mpz::mpz_int> *m_cache;

  std::ostream m_out;
  std::vector<bool> m_isUniversalVar;

 public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  QbfCounter(const OptionQbfCounter &options, ProblemManagerQbf *initProblem,
             std::ostream &out)
      : m_problem(initProblem), m_out(nullptr) {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());
    m_out.setstate(out.rdstate());

    m_out << "c [QBF COUNTER]" << options << "\n";

    // we create and init the SAT solver.
    m_solver = WrapperSolver::makeWrapperSolver(options.optionSolver, m_out);
    m_solver->initSolver(*m_problem);
    m_solver->setNeedModel(true);

    // we initialize the object that will give info about the problem.
    m_specs = SpecManager::makeSpecManager(options.optionSpecManager,
                                           *m_problem, m_out);

    // we initialize the object used to compute score and partition.
    m_heuristic = BranchingHeuristic::makeBranchingHeuristic(
        options.optionBranchingHeuristic, m_specs, m_solver, m_out);

    // init the cache manager.
    m_cache = CacheManager<mpz::mpz_int>::makeCacheManager(
        options.optionCacheManager, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    m_optCached = options.optionCacheManager.isActivated;
    m_nbDecisionNode = m_nbSplit = m_nbCallCall = 0;
    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);

    m_currentPrioritySet.resize(m_specs->getNbVariable() + 1, true);
    m_isDecisionVariable.resize(m_specs->getNbVariable() + 1, true);

    std::vector<Block> &qblocks = initProblem->getQBlocks();
    m_out << "c [QBF COUNTER] Number of blocks: " << qblocks.size() << '\n';
    m_isUniversalVar.resize(m_specs->getNbVariable() + 1, false);
    m_varBlockLevel.resize(m_specs->getNbVariable() + 1, 0);
    for (unsigned i = 0; i < qblocks.size(); i++) {
      for (auto &v : qblocks[i].variables) {
        m_isUniversalVar[v] = qblocks[i].isUniversal;
        m_varBlockLevel[v] = i;
      }
    }

    m_out << "c\n";
  }  // constructor

  /**
     Destructor.
   */
  ~QbfCounter() {
    delete m_problem;
    delete m_solver;
    delete m_specs;
    delete m_heuristic;
    delete m_cache;
  }  // destructor

 private:
  /**
   * Print out information about the solving process.
   *
   * @param[in] out, the stream we use to print out information.
   */
  inline void showInter(std::ostream &out) {
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
  inline void separator(std::ostream &out) {
    out << "c ";
    for (int i = 0; i < NB_SEP_QBF_MC; i++) out << "-";
    out << "\n";
  }  // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out) {
    separator(out);
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node"
        << "|\n";
    separator(out);
  }  // showHeader

  /**
     Print out information when it is requiered.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out) {
    if (!(m_nbCallCall & (MASK_HEADER))) showHeader(out);
    if (m_nbCallCall && !(m_nbCallCall & MASK_SHOWRUN_MC)) showInter(out);
  }  // showRun

  /**
     Print out the final stat.

     @param[in] out, the stream we use to print out information.
   */
  inline void printFinalStats(std::ostream &out) {
    separator(out);
    out << "c\n";
    out << "c \033[1m\033[31mStatistics \033[0m\n";
    out << "c \033[33mCompilation Information\033[0m\n";
    out << "c Number of recursive call: " << m_nbCallCall << "\n";
    out << "c Number of split formula: " << m_nbSplit << "\n";
    out << "c Number of decision: " << m_nbDecisionNode << "\n";
    out << "c\n";
    m_cache->printCacheInformation(out);
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  }  // printFinalStat

  /**
     Initialize the assumption in order to compute compiled formula under this
     one.

     @param[in] assums, the assumption
  */
  inline void initAssumption(std::vector<Lit> &assums) {
    m_solver->restart();
    m_solver->popAssumption(m_solver->getAssumption().size());
    m_solver->setAssumption(assums);
  }  // initAssumption

  /**
     Decide if the cache is realized or not.
   */
  bool cacheIsActivated(std::vector<Var> &connected) {
    if (!m_optCached) return false;
    return m_cache->isActivated(connected.size());
  }  // cacheIsActivated

  /**
   * Call the CNF formula into a FBDD.
   *
   * @param[in] setOfVar, the current set of considered variables
   * @param[in] unitsLit, the set of unit literal detected at this level
   * @param[in] freeVariable, the variables which become free
   * @param[in] out, the stream we use to print out information.
   *
   * \return an element of type U that sums up the given CNF sub-formula
   * using a DPLL style algorithm with an operation manager.
   */
  mpz::mpz_int compute_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
                        std::vector<Var> &freeVariable, std::ostream &out) {
    showRun(out);
    m_nbCallCall++;
    if (!m_solver->solve(setOfVar)) return 0;

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals

    // the universal part cannot propagate unit literal.
    for (auto &l : unitsLit)
      if (!m_solver->isInAssumption(l.var()) && m_isUniversalVar[l.var()])
        return 0;

    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);

    unsigned countUniv = 0;
    for (auto &v : setOfVar)
      if (m_isUniversalVar[v] && !m_solver->varIsAssigned(v)) countUniv++;
    for (auto &v : freeVariable)
      if (m_isUniversalVar[v] && !m_solver->varIsAssigned(v)) countUniv--;

    // consider each connected component.
    if (nbComponent) {
      mpz::mpz_int result = 1, tmpCount;
      m_nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent && result != 0; cp++) {
        std::vector<Var> &connected = varConnected[cp];

        unsigned countLocalUniv = 0;
        for (auto &v : connected)
          if (m_isUniversalVar[v]) countLocalUniv++;

        bool cacheActivated = cacheIsActivated(connected);
        TmpEntry<mpz::mpz_int> cb = cacheActivated
                                        ? m_cache->searchInCache(connected)
                                        : NULL_CACHE_ENTRY;
        if (cacheActivated && cb.defined)
          tmpCount = cb.getValue();
        else {
          // recursive call
          tmpCount = computeDecisionNode(connected, out);
          if (cacheActivated) m_cache->addInCache(cb, tmpCount);
        }

        // rescale regarding the other component.
        for (unsigned i = 0; i < countUniv - countLocalUniv; i++)
          tmpCount *= tmpCount;
        result *= tmpCount;
      }

      m_specs->postUpdate(unitsLit);
      return result;
    }  // else we have a tautology

    m_specs->postUpdate(unitsLit);
    return 1;
  }  // compute_

  /**
   * @brief Set the Current Priority.
   *
   * @param cutSet is the set of variables that become decision variables.
   */
  inline void setCurrentPriority(std::vector<Var> &cutSet) {
    for (auto &v : cutSet)
      if (m_isDecisionVariable[v]) m_currentPrioritySet[v] = true;
  }  // setCurrentPriority

  /**
   * @brief Unset the Current Priority.
   *
   * @param cutSet is the set of variables that become decision variables.
   */
  inline void unsetCurrentPriority(std::vector<Var> &cutSet) {
    for (auto &v : cutSet)
      if (m_isDecisionVariable[v]) m_currentPrioritySet[v] = false;
  }  // setCurrentPriority

  /**
     This function select a variable and compile a decision node.

     @param[in] connected, the set of variable present in the current problem.
     @param[in] out, the stream whare are printed out the logs.

     \return the compiled formula.
  */
  mpz::mpz_int computeDecisionNode(std::vector<Var> &connected,
                                   std::ostream &out) {
    unsigned level = m_varBlockLevel[connected[0]];
    std::vector<Var> candidateVar = {connected[0]};
    for (auto &v : connected)
      if (m_varBlockLevel[v] == level)
        candidateVar.push_back(v);
      else if (m_varBlockLevel[v] < level) {
        candidateVar.resize(0);
        level = m_varBlockLevel[v];
        candidateVar.push_back(v);
      }
    assert(candidateVar.size());

    ListLit lits;
    m_heuristic->selectLitSet(candidateVar, m_currentPrioritySet, lits);
    assert(lits.size() == 0);

    m_nbDecisionNode++;

    // count on the formula when lits[0] is assigned true and false.
    assert(lits.size() == 1);
    DataBranch<mpz::mpz_int> b[2];

    unsigned nb = 0;

    mpz::mpz_int result = 0;
    m_solver->pushAssumption(lits[0]);
    b[0].d = compute_(connected, b[0].unitLits, b[0].freeVars, out);
    m_solver->popAssumption();
    if (m_isUniversalVar[lits[0].var()] && b[0].d == 0) return 0;

    m_solver->pushAssumption(~lits[0]);
    b[1].d = compute_(connected, b[1].unitLits, b[1].freeVars, out);
    m_solver->popAssumption();
#if 0
    if (m_isUniversalVar[v]) {
      mpz::mpz_int freeScale = 2;
      for (unsigned i = 0; i < univVar.size() - 1; i++) freeScale *= 2;

      mpz::mpz_int c0 = b[0].d, c1 = b[1].d;
      unsigned countUnivFree0 = 0, countUnivFree1 = 0;
      for (auto &v : b[0].freeVars)
        if (m_isUniversalVar[v])
          countUnivFree0++;
        else
          c0 *= freeScale;

      for (auto &v : b[1].freeVars)
        if (m_isUniversalVar[v])
          countUnivFree1++;
        else
          c1 *= freeScale;

      for (unsigned i = 0; i < countUnivFree0; i++) c0 *= c0;
      for (unsigned i = 0; i < countUnivFree1; i++) c1 *= c1;

      result = c0 * c1;
    } else {
      result += b[0].d * m_problem->computeWeightUnitFree<mpz::mpz_int>(b[0]);
      result += b[1].d * m_problem->computeWeightUnitFree<mpz::mpz_int>(b[1]);
    }
#endif
    return result;
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
  mpz::mpz_int compute(std::vector<Var> &setOfVar, std::ostream &out,
                       bool warmStart = true) {
    if (m_problem->isUnsat() ||
        (warmStart && !m_solver->warmStart(29, 11, setOfVar, m_out))) {
      return 0;
    }
    DataBranch<mpz::mpz_int> b;
    b.d = compute_(setOfVar, b.unitLits, b.freeVars, out);
    return b.d * m_problem->computeWeightUnitFree<mpz::mpz_int>(b);
  }  // compute

 public:
  /**
     Given an assumption, we compute the number of models.  That is different
     from the query strategy, where we first compute and then condition the
     computed structure.

     @param[in] setOfVar, the set of variables of the considered problem.
     @param[in] assumption, the set of literals we want to assign.
     @param[in] out, the stream where are print out the log.

     \return the number of models when the formula is simplified by the given
     assumption.
   */
  mpz::mpz_int count(std::vector<Var> &setOfVar, std::vector<Lit> &assumption,
                     std::ostream &out) {
    initAssumption(assumption);

    // get the unit not in setOfVar.
    std::vector<Lit> shadowUnits;
    m_stampIdx++;
    for (auto &v : setOfVar) m_stampVar[v] = m_stampIdx;
    for (auto &l : assumption)
      if (m_stampVar[l.var()] != m_stampIdx) shadowUnits.push_back(l);

    m_specs->preUpdate(shadowUnits);
    mpz::mpz_int result = compute(setOfVar, out, false);
    m_specs->postUpdate(shadowUnits);

    assert(0);
    return 1;
  }  // count

  /**
     Run the DPLL style algorithm with the operation manager.

     @param[in] vm, the set of options.
   */
  mpz::mpz_int run() {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);

    mpz::mpz_int result = compute(setOfVar, m_out);
    printFinalStats(m_out);
    return result;
  }  // run
};
}  // namespace d4
