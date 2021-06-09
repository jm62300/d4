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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <bits/stdint-uintn.h>
#include <boost/program_options.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sys/types.h>

#include "src/caching/Cache.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/heuristics/PartitioningHeuristic.hpp"
#include "src/heuristics/PhaseHeuristic.hpp"
#include "src/heuristics/ScoringMethod.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/utils/MemoryStat.hpp"

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"

#define NB_SEP_MC 118
#define MASK_SHOWRUN_MC ((2 << 13) - 1)
#define WIDTH_PRINT_COLUMN_MC 12
#define MASK_HEADER 1048575

namespace d4 {

namespace po = boost::program_options;
template <class T> class Counter;

template <class T> class MaxSharpSAT : public MethodManager, public Counter<T> {
  enum TypeDecision { NO_DEC, EXIST_DEC, MAX_DEC };

private:
  bool optDomConst;
  bool optReversePolarity;

  unsigned nbCallCall;
  unsigned nbSplit;
  unsigned callPartitioner;
  unsigned nbDecisionNode;
  unsigned optCached;
  unsigned m_stampIdx;
  unsigned m_limitUpdate;
  unsigned m_limitUpdateCounter;
  bool m_staticLimit;
  bool m_isProjectedMode;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> clauses;

  std::vector<unsigned long> nbTestCacheVarSize;
  std::vector<unsigned long> nbPosHitCacheVarSize;
  std::vector<TypeDecision> m_decisionStatus;
  std::vector<bool> m_isDecisionVarible;

  ProblemManager *m_problem;
  WrapperSolver *m_solver;
  SpecManager *m_specs;
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  PartitioningHeuristic *m_hCutSet;

  TmpEntry<T> NULL_CACHE_ENTRY;
  Cache<T> *m_cache;

  std::ostream m_out;
  bool m_panicMode;

  unsigned limitNbVarCache;
  unsigned limitNbVarCacheDynamic;
  double ratioDynamicLimit;

public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  MaxSharpSAT(po::variables_map &vm, std::string &meth, bool isFloat,
              ProblemManager *initProblem, std::ostream &out,
              LastBreathPreproc &lastBreath)
      : m_problem(initProblem), m_out(nullptr) {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());

    // we create the SAT solver.
    m_solver = WrapperSolver::makeWrapperSolver(vm, m_out);
    assert(m_solver);
    m_panicMode = lastBreath.panic;
    m_solver->initSolver(*m_problem);
    m_solver->setCountConflict(lastBreath.countConflict, 1,
                               m_problem->getNbVar());
    m_solver->setNeedModel(true);

    // we initialize the object that will give info about the problem.
    m_specs = SpecManager::makeSpecManager(vm, *m_problem, m_out);
    assert(m_specs);

    // we initialize the object used to compute score and partition.
    m_hVar = ScoringMethod::makeScoringMethod(vm, *m_specs, *m_solver, m_out);
    m_hPhase =
        PhaseHeuristic::makePhaseHeuristic(vm, *m_specs, *m_solver, m_out);

    // specify which variables are decisions, and which are not.
    m_decisionStatus.clear();
    m_isDecisionVarible.clear();

    m_isDecisionVarible.resize(m_problem->getNbVar() + 1, false);
    m_decisionStatus.resize(m_problem->getNbVar() + 1, NO_DEC);
    for (auto v : m_problem->getMaxVar()) {
      m_isDecisionVarible[v] = true;
      m_decisionStatus[v] = MAX_DEC;
    }
    for (auto v : m_problem->getIndVar()) {
      m_isDecisionVarible[v] = true;
      m_decisionStatus[v] = EXIST_DEC;
    }

    // no partitioning heuristic for the moment.
    m_hCutSet = PartitioningHeuristic::makePartitioningHeuristicNone(m_out);

    assert(m_hVar && m_hPhase && m_hCutSet);
    m_cache = new Cache<T>(vm, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    optCached = vm["cache-activated"].as<bool>();
    callPartitioner = 0;
    nbDecisionNode = nbSplit = nbCallCall = 0;

    m_limitUpdate = vm["cache-limit-update-frequency"].as<unsigned>();
    m_staticLimit = vm["cache-limit-static"].as<bool>();
    m_limitUpdateCounter = 0;

    if (m_staticLimit) {
      ratioDynamicLimit = 1;
      limitNbVarCache = m_problem->getNbVar();
    } else {
      ratioDynamicLimit = vm["cache-limit-ratio-number-variable"].as<double>();
      limitNbVarCache = vm["cache-limit-number-variable"].as<unsigned>();
    }
    limitNbVarCacheDynamic = limitNbVarCache;

    m_out << "c [CONSTRUCTOR] Limit number of variables for caching: "
          << "static(" << m_staticLimit << ") "
          << "limit(" << limitNbVarCache << ") "
          << "ratio(" << ratioDynamicLimit << ") "
          << "limitDyn(" << limitNbVarCacheDynamic << ") "
          << "freq update(" << m_limitUpdate << ") "
          << "\n";

    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);
    nbTestCacheVarSize.resize(m_specs->getNbVariable() + 1, 0);
    nbPosHitCacheVarSize.resize(m_specs->getNbVariable() + 1, 0);
    m_out << "c\n";
  } // constructor

  /**
     Destructor.
   */
  ~MaxSharpSAT() {
    delete m_problem;
    delete m_solver;
    delete m_specs;
    delete m_hVar;
    delete m_hPhase;
    delete m_hCutSet;
    delete m_cache;
  } // destructor

private:
  /**
     Expel from a set of variables the ones they are marked as being decidable.

     @param[out] vars, the set of variables we search to filter.

     @param[in] isDecisionvariable, a type decision vector that marks as true
     decision variables.
   */
  void expelNoDesionVar(std::vector<Var> &vars,
                        std::vector<TypeDecision> &isDecisionVariable) {
    if (!m_isProjectedMode)
      return;
    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++)
      if (isDecisionVariable[vars[i]] != NO_DEC)
        vars[j++] = vars[i];
    vars.resize(j);
  } // expelNoDesionVar

  /**
     Compute the current priority set.

     @param[in] connected, the current component.
     @param[in] priorityVar, the current priority variables.
     @param[out] currPriority, the intersection of the two previous sets.
  */
  inline void computePrioritySubSet(std::vector<Var> &connected,
                                    std::vector<Var> &priorityVar,
                                    std::vector<Var> &currPriority) {
    currPriority.clear();
    m_stampIdx++;
    for (auto &v : connected)
      m_stampVar[v] = m_stampIdx;
    for (auto &v : priorityVar)
      if (m_stampVar[v] == m_stampIdx && !m_specs->varIsAssigned(v))
        currPriority.push_back(v);
  } // computePrioritySet

  /**
     Print out information about the solving process.

     @param[in] out, the stream we use to print out information.
  */
  inline void showInter(std::ostream &out) {
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << nbCallCall << std::fixed
        << std::setprecision(2) << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << getTimer() << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cache->getNbPositiveHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->getNbNegativeHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->usedMemory()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << nbSplit << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << nbDecisionNode << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << callPartitioner

        << "|\n";
  } // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out) {
    out << "c ";
    for (int i = 0; i < NB_SEP_MC; i++)
      out << "-";
    out << "\n";
  } // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out) {
    separator(out);
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#compile"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#cutter"
        << "|\n";
    separator(out);
  } // showHeader

  /**
     Print out information when it is requiered.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out) {
    if (!(nbCallCall & (MASK_HEADER)))
      showHeader(out);
    if (nbCallCall && !(nbCallCall & MASK_SHOWRUN_MC))
      showInter(out);
  } // showRun

  /**
     Print out the final stat.

     @param[in] out, the stream we use to print out information.
   */
  inline void printFinalStats(std::ostream &out) {
    separator(out);
    out << "c\n";
    out << "c \033[1m\033[31mStatistics \033[0m\n";
    out << "c \033[33mCompilation Information\033[0m\n";
    out << "c Number of recursive call: " << nbCallCall << "\n";
    out << "c Number of split formula: " << nbSplit << "\n";
    out << "c Number of decision: " << nbDecisionNode << "\n";
    out << "c Number of paritioner calls: " << callPartitioner << "\n";
    out << "c\n";
    m_cache->printCacheInformation(out);
    if (m_hCutSet) {
      out << "c\n";
      m_hCutSet->displayStat(out);
    }
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  } // printFinalStat

  /**
     Initialize the assumption in order to compute compiled formula under this
     one.

     @param[in] assums, the assumption
  */
  inline void initAssumption(std::vector<Lit> &assums) {
    m_solver->restart();
    m_solver->popAssumption(m_solver->getAssumption().size());
    m_solver->setAssumption(assums);
  } // initAssumption

  /**
     Decide if the cache is realized or not.
   */
  bool cacheIsActivated(std::vector<Var> &connected) {
    if (!optCached)
      return false;
    if (connected.size() < limitNbVarCache)
      return true;
    if (connected.size() < limitNbVarCacheDynamic)
      return true;
    return false;
  } // cacheIsActivated

  /**
     Update the dynamic limit.
   */
  void updateDynamicLimit() {
    limitNbVarCacheDynamic = 0;
    if (m_staticLimit)
      return;

    for (unsigned i = 0; i < nbPosHitCacheVarSize.size(); i++) {
      if (nbPosHitCacheVarSize[i])
        limitNbVarCacheDynamic = i;
      nbPosHitCacheVarSize[i] >>= 1;
    }

    limitNbVarCacheDynamic *= ratioDynamicLimit;
    m_out << "c Update dynamic limit: " << limitNbVarCacheDynamic << "/"
          << limitNbVarCache << "\n";
  } // updateDynamicLimit

  /**
     Call the CNF formula into a D-FPiBDD.

     @param[in] setOfVar, the current set of considered variables
     @param[in] unitsLit, the set of unit literal detected at this level
     @param[in] freeVariable, the variables which become free
     @param[in] priority, select in priority these variable to the next decision
     node
     @param[in] out, the stream we use to print out information.

     \return an element of type U that sums up the given CNF sub-formula using a
     DPLL style algorithm with an operation manager.
  */
  T compute_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
             std::vector<Var> &freeVariable, std::vector<Var> &priorityVar,
             std::ostream &out) {
    showRun(out);
    nbCallCall++;

    if (!m_solver->solve(setOfVar))
      return T(0);

    m_solver->whichAreUnits(setOfVar, unitsLit); // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<Var> reallyPresent;
    std::vector<std::vector<Var>> varConnected;

    int nbComponent = m_specs->computeConnectedComponent(
        varConnected, setOfVar, freeVariable, reallyPresent);
    expelNoDesionVar(freeVariable, m_decisionStatus);

    if (++m_limitUpdateCounter > m_limitUpdate) {
      updateDynamicLimit();
      m_limitUpdateCounter = 0;
    }

    // consider each connected component.
    T ret = T(1);
    if (nbComponent) {
      nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        bool cacheActivated = cacheIsActivated(connected);

        TmpEntry<T> cb = cacheActivated ? m_cache->searchInCache(connected)
                                        : NULL_CACHE_ENTRY;

        if (cacheActivated)
          nbTestCacheVarSize[connected.size()]++;
        if (cacheActivated && cb.defined) {
          nbPosHitCacheVarSize[connected.size()]++;
          ret = ret * cb.getValue();
        } else {
          // recursive call
          std::vector<Var> currPriority;
          computePrioritySubSet(connected, priorityVar, currPriority);

          T curr = computeDecisionNode(connected, currPriority, out);
          if (cacheActivated)
            m_cache->addInCache(cb, curr);
          ret = ret * curr;
        }
      }
    } // else we have a tautology

    m_specs->postUpdate(unitsLit);
    return ret;
  } // compute_

  /**
     This function select a variable and compile a decision node.

     @param[in] connected, the set of variable present in the current problem.
     @param[in] priorityVar, a list of variable we want to branch first.

     \return the compiled formula.
  */
  T computeDecisionNode(std::vector<Var> &connected, std::vector<Var> &priority,
                        std::ostream &out) {
    if (!priority.size() && m_hCutSet->isReady(connected)) {
      m_hCutSet->computeCutSet(connected, priority);
      callPartitioner++;
    }

    // search the next variable to branch on
    std::vector<Var> &inVars = (priority.size()) ? priority : connected;
    Var v = m_hVar->selectVariable(inVars, *m_specs, m_isDecisionVarible);

    if (v == var_Undef)
      return T(1);

    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = compute_(connected, b[0].unitLits, b[0].freeVars, priority, out);
    m_solver->popAssumption();

    if (m_solver->isInAssumption(l))
      b[1].d = 0;
    else if (m_solver->isInAssumption(~l))
      b[1].d = compute_(connected, b[1].unitLits, b[1].freeVars, priority, out);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = compute_(connected, b[1].unitLits, b[1].freeVars, priority, out);
      m_solver->popAssumption();
    }

    b[0].d *= m_problem->computeWeightUnitFree<T>(b[0].unitLits, b[0].freeVars);
    b[1].d *= m_problem->computeWeightUnitFree<T>(b[1].unitLits, b[1].freeVars);
    return b[0].d + b[1].d;
  } // computeDecisionNode

  /**
     Compute U using the trace of a SAT solver.

     @param[in] setOfVar, the set of variables of the considered problem.
     @param[in] out, the stream are is print out the logs.
     @param[in] warmStart, to activate/deactivate the warm start strategy.
     /!\ When the warm strat is activated we the assumptions are reset.

     \return an element of type U that sums up the given CNF formula using a
     DPLL style algorithm with an operation manager.
  */
  T compute(std::vector<Var> &setOfVar, std::ostream &out,
            bool warmStart = true) {
    if (m_problem->isUnsat() || (warmStart && !m_panicMode &&
                                 !m_solver->warmStart(29, 11, setOfVar, m_out)))
      return T(0);

    std::vector<Var> priorityVar;
    DataBranch<T> b;
    b.d = compute_(setOfVar, b.unitLits, b.freeVars, priorityVar, out);
    return b.d * m_problem->computeWeightUnitFree<T>(b.unitLits, b.freeVars);
  } // compute

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
  T count(std::vector<Var> &setOfVar, std::vector<Lit> &assumption,
          std::ostream &out) {
    initAssumption(assumption);

    // get the unit not in setOfVar.
    std::vector<Lit> shadowUnits;
    m_stampIdx++;
    for (auto &v : setOfVar)
      m_stampVar[v] = m_stampIdx;
    for (auto &l : assumption)
      if (m_stampVar[l.var()] != m_stampIdx)
        shadowUnits.push_back(l);

    m_specs->preUpdate(shadowUnits);
    T result = compute(setOfVar, out, false);
    m_specs->postUpdate(shadowUnits);

    return result;
  } // count

  /**
   * @brief Search for the instanciation of the variable of
   * m_problem->getMaxVar() that maximize the number of the remaining variables
   * where the variables not belonging to m_problem->getIndVar() are
   * existantially quatified.
   *
   * @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm) {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++)
      setOfVar.push_back(i);

    T result = compute(setOfVar, m_out);
    printFinalStats(m_out);
    std::cout << "s " << result << "\n";
  } // run
};
} // namespace d4
