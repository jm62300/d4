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

namespace d4 {
namespace po = boost::program_options;
template <class T> class Counter;

template <class T> class MaxSharpSAT : public MethodManager {
  enum TypeDecision { NO_DEC, EXIST_DEC, MAX_DEC };

  struct MaxSharpSatResult {
    unsigned long int *valuation;
    T count;
  };

private:
  const unsigned NB_SEP = 105;

  bool optDomConst;
  bool optReversePolarity;

  unsigned nbCallCall;
  unsigned nbSplit;
  unsigned nbDecisionNode;
  unsigned optCached;
  unsigned m_stampIdx;
  bool m_isProjectedMode;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> clauses;

  std::vector<bool> m_isDecisionVarible;
  std::vector<bool> m_isMaxDecisionVarible;
  T m_maxCount = T(0);

  ProblemManager *m_problem;
  WrapperSolver *m_solver;
  SpecManager *m_specs;
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;

  TmpEntry<T> NULL_CACHE_ENTRY;
  Cache<T> *m_cache;

  std::ostream m_out;
  bool m_panicMode;

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
    m_isDecisionVarible.clear();
    m_isMaxDecisionVarible.clear();

    m_isDecisionVarible.resize(m_problem->getNbVar() + 1, false);
    for (auto v : m_problem->getIndVar())
      m_isDecisionVarible[v] = true;

    m_isMaxDecisionVarible.resize(m_problem->getNbVar() + 1, false);
    for (auto v : m_problem->getMaxVar())
      m_isMaxDecisionVarible[v] = true;

    // no partitioning heuristic for the moment.
    assert(m_hVar && m_hPhase);
    m_cache = new Cache<T>(vm, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    optCached = vm["cache-activated"].as<bool>();
    nbDecisionNode = nbSplit = nbCallCall = 0;

    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);
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
    delete m_cache;
  } // destructor

private:
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
        << std::setw(WIDTH_PRINT_COLUMN_MC) << nbDecisionNode << "|\n";
  } // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out) {
    out << "c ";
    for (int i = 0; i < NB_SEP; i++)
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
    out << "c\n";
    m_cache->printCacheInformation(out);
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  } // printFinalStat

  /**
   * Expel from a set of variables the ones they are marked as being decidable.
   * @param[out] vars, the set of variables we search to filter.
   * @param[in] isDecisionvariable, a type decision vector that marks as true
   * decision variables.
   */
  void expelNoDecisionVar(std::vector<Var> &vars,
                          std::vector<bool> &isDecisionVariable) {
    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++)
      if (isDecisionVariable[vars[i]])
        vars[j++] = vars[i];
    vars.resize(j);
  } // expelNoDecisionVar

  /**
   * Expel from a set of variables the ones they are marked as being decidable.
   * @param[out] vars, the set of variables we search to filter.
   * @param[in] isDecisionvariable, a type decision vector that marks as true
   * decision variables.
   */
  void expelNoDecisionLit(std::vector<Lit> &lits,
                          std::vector<bool> &isDecisionVariable) {
    unsigned j = 0;
    for (unsigned i = 0; i < lits.size(); i++)
      if (isDecisionVariable[lits[i].var()])
        lits[j++] = lits[i];
    lits.resize(j);
  } // expelNoDecisionLit

  /**
   * @brief Search for a valuation of the max variables that maximizes the
   * number of models on the remaning formula where some variables are forget.
   *
   * @param setOfVar, the current set of considered variables.
   * @param unitsLit, the set of unit literal detected at this level.
   * @param freeVariable, the variables which become free decision node.
   * @param out, the stream we use to print out logs.
   * @param result, the strucre where is solved the result.
   */
  void searchMaxValuation(std::vector<Var> &setOfVar,
                          std::vector<Lit> &unitsLit,
                          std::vector<Var> &freeVariable, std::ostream &out,
                          MaxSharpSatResult &result) {
    showRun(out);
    nbCallCall++;

    // is the problem still satisifiable?
    if (!m_solver->solve(setOfVar)) {
      result.count = T(0);
      return;
    }

    m_solver->whichAreUnits(setOfVar, unitsLit); // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<Var> reallyPresent;
    std::vector<std::vector<Var>> varConnected;

    int nbComponent = m_specs->computeConnectedComponent(
        varConnected, setOfVar, freeVariable, reallyPresent);
    expelNoDecisionVar(freeVariable, m_isDecisionVarible);

    // consider each connected component.
    result.count = T(1);
    if (nbComponent) {
      nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        TmpEntry<T> cb = m_cache->searchInCache(connected);

        if (cb.defined)
          result.count = result.count * cb.getValue();
        else {
          MaxSharpSatResult tmpResult;
          searchMaxSharpSatDecision(connected, out, tmpResult);
          m_cache->addInCache(cb, tmpResult.count);
          result.count = result.count * tmpResult.count;
        }
      }
    } // else we have a tautology

    m_solver->showTrail();
    std::cout << "=> " << result.count << "\n";

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit, m_isDecisionVarible);
  } // searchMaxValuation

  /**
   * This function select a variable and compile a decision node.
   *
   * @param[in] connected, the set of variable present in the current problem.
   * @param[in] out, the stream we use to print out logs.
   * @param[out] result, the best solution found.
   *
   * \return the compiled formula.
   */
  void searchMaxSharpSatDecision(std::vector<Var> &connected, std::ostream &out,
                                 MaxSharpSatResult &result) {
    // search the next variable to branch on
    Var v = m_hVar->selectVariable(connected, *m_specs, m_isMaxDecisionVarible);

    if (v == var_Undef) {
      std::vector<Lit> unitsLit;
      std::vector<Var> freeVar;
      result.count = countInd_(connected, unitsLit, freeVar, out);
      result.count *= m_problem->computeWeightUnitFree<T>(unitsLit, freeVar);

      if (result.count > m_maxCount) {
        m_maxCount = result.count;
        std::cout << "=> " << m_maxCount << "\n";
      }
      return;
    }

    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = searchMaxValuation(connected, b[0].unitLits, b[0].freeVars, out);
    m_solver->popAssumption();

    if (m_solver->isInAssumption(l))
      b[1].d = 0;
    else if (m_solver->isInAssumption(~l))
      b[1].d = searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out);
      m_solver->popAssumption();
    }

    b[0].d *= m_problem->computeWeightUnitFree<T>(b[0].unitLits, b[0].freeVars);
    b[1].d *= m_problem->computeWeightUnitFree<T>(b[1].unitLits, b[1].freeVars);

    result.count = (b[0].d > b[1].d) ? b[0].d : b[1].d;

    if (result.count > m_maxCount) {
      m_maxCount = result.count;
      std::cout << "=> " << m_maxCount << "\n";
    }
  } // searchMaxSharpSatDecision

  /**
   * Count the number of projected models.
   *
   * @param[in] setOfVar, the current set of considered variables
   * @param[in] unitsLit, the set of unit literal detected at this level
   * @param[in] freeVariable, the variables which become free decision node
   * @param[in] out, the stream we use to print out logs.
   *
   * \return an element of type U that sums up the given CNF sub-formula using
   * a DPLL style algorithm with an operation manager.
   */
  T countInd_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
              std::vector<Var> &freeVariable, std::ostream &out) {
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
    expelNoDecisionVar(freeVariable, m_isDecisionVarible);

    // consider each connected component.
    T ret = T(1);
    if (nbComponent) {
      nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        TmpEntry<T> cb = m_cache->searchInCache(connected);

        if (cb.defined)
          ret = ret * cb.getValue();
        else {
          T curr = countIndDecisionNode(connected, out);
          m_cache->addInCache(cb, curr);
          ret = ret * curr;
        }
      }
    } // else we have a tautology

    m_specs->postUpdate(unitsLit);
    return ret;
  } // countInd_

  /**
   * This function select a variable and compile a decision node.
   *
   * @param[in] connected, the set of variable present in the current problem.
   * @param[in] out, the stream we use to print out logs.
   *
   * \return the compiled formula.
   */
  T countIndDecisionNode(std::vector<Var> &connected, std::ostream &out) {
    // search the next variable to branch on
    Var v = m_hVar->selectVariable(connected, *m_specs, m_isDecisionVarible);

    if (v == var_Undef)
      return T(1);

    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = countInd_(connected, b[0].unitLits, b[0].freeVars, out);
    m_solver->popAssumption();

    if (m_solver->isInAssumption(l))
      b[1].d = 0;
    else if (m_solver->isInAssumption(~l))
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out);
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
     /!\ When the warm start is activated we the assumptions are reset.

     \return an element of type U that sums up the given CNF formula using a
     DPLL style algorithm with an operation manager.
  */
  void compute(std::vector<Var> &setOfVar, std::ostream &out,
               MaxSharpSatResult &result, bool warmStart = true) {
    if (m_problem->isUnsat() || (warmStart && !m_panicMode &&
                                 !m_solver->warmStart(29, 11, setOfVar, m_out)))
      result.count = T(0);

    DataBranch<T> b;
    b.d = countInd_(setOfVar, b.unitLits, b.freeVars, out);
    result.count =
        b.d * m_problem->computeWeightUnitFree<T>(b.unitLits, b.freeVars);
  } // compute

public:
  /**
   * @brief Search for the instanciation of the variables of
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

    MaxSharpSatResult result;
    compute(setOfVar, m_out, result);
    printFinalStats(m_out);
    std::cout << "s " << result.count << "\n";
  } // run
};
} // namespace d4
