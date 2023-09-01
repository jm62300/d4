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

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <boost/program_options.hpp>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <ios>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/heuristics/OptionBranchingHeuristic.hpp"
#include "src/heuristics/PartitioningHeuristic.hpp"
#include "src/heuristics/PhaseHeuristic.hpp"
#include "src/heuristics/PhaseHeuristicPolarity.hpp"
#include "src/heuristics/ScoringMethod.hpp"
#include "src/methods/nnf/Node.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/OptionSolver.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/OptionSpecManager.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/utils/MemoryStat.hpp"

#define TEST 0

namespace d4 {
namespace po = boost::program_options;
template <class T>
class Counter;

template <class T>
class ExistRandomExist : public MethodManager {
  enum TypeDecision { NO_DEC, EXIST_DEC, MAX_DEC };

  struct MaxSharpSatResult {
    T count;
    u_int8_t *valuation;

    MaxSharpSatResult() : count(T(0)), valuation(NULL) {}
    MaxSharpSatResult(const T c, u_int8_t *v) : count(c), valuation(v) {}

    void display(unsigned size) {
      assert(valuation);
      for (unsigned i = 0; i < size; i++) std::cout << (int)valuation[i] << " ";
      std::cout << "\n";
    }
  };

 private:
  const unsigned NB_SEP = 222;

  bool optDomConst;
  bool optReversePolarity;

  unsigned m_nbCallCall;
  unsigned m_nbCallProj;
  unsigned m_nbSplitMax = 0;
  unsigned m_nbSplitInd = 0;
  unsigned m_nbDecisionNode;
  unsigned m_optCached;
  unsigned m_stampIdx;
  unsigned m_nbCutUpperBoundMaxPart = 0;
  unsigned m_nbCutUpperBoundIndPart = 0;
  unsigned m_nbPureMax = 0;
  unsigned m_nbPureInd = 0;

  bool m_hasBeenStop = false;
  bool m_isUnderAnd = false;
  bool m_greedyInitActivated;

  unsigned m_nbCallIndSaved = 0;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> clauses;

  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_isProjectedVariable;
  std::vector<bool> m_isMaxDecisionVariable;
  std::vector<unsigned> m_redirectionPos;
  unsigned m_countUpdateMaxCount = 0;

  const unsigned c_sizePage = 1 << 18;
  std::vector<u_int8_t *> m_memoryPages;
  unsigned m_posInMemoryPages;
  unsigned m_sizeArray;

  ProblemManager *m_problem;
  WrapperSolver *m_solver;
  SpecManager *m_specs;
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  PhaseHeuristic *m_hPhaseInd;

  CacheManager<T> *m_cacheInd;
  CacheManager<MaxSharpSatResult> *m_cacheMax;

  std::ostream m_out;
  bool m_panicMode;

  double m_threshold = -1;
  bool m_stopProcess = false;
  bool m_andDig = false;
  bool m_cutUpperMax = false;
  bool m_componentOnProjected = false;
  std::string m_heuristicMax = "none";
  unsigned m_heuristicMaxRdm = 0;

  MaxSharpSatResult m_scale = {T(1), NULL};
  MaxSharpSatResult m_maxCount = {T(0), NULL};

 public:
  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  ExistRandomExist(po::variables_map &vm, const std::string &meth,
                   ProblemManager *initProblem, std::ostream &out)
      : m_problem(initProblem), m_out(nullptr) {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());
    m_cutUpperMax = vm["ere-cut-upperBound"].as<bool>();
    m_componentOnProjected = vm["ere-component-on-projected"].as<bool>();
    m_heuristicMax = vm["maxsharpsat-heuristic-phase"].as<std::string>();
    m_greedyInitActivated = vm["maxsharpsat-option-greedy-init"].as<bool>();
    m_optCached = vm["cache-activated"].as<bool>();
    m_heuristicMaxRdm = vm["maxsharpsat-heuristic-phase-random"].as<unsigned>();
    m_threshold = vm["maxsharpsat-threshold"].as<double>();
    m_andDig = vm["maxsharpsat-option-and-dig"].as<bool>();

    m_out << "c [CONSTRUCTOR ERE] Heuristic on MAX variables: "
          << m_heuristicMax << "\n"
          << "c [CONSTRUCTOR ERE] Use random on MAX variables: "
          << m_heuristicMaxRdm << "\n"
          << "c [CONSTRUCTOR ERE] Threshold: " << m_threshold << "\n"
          << "c [CONSTRUCTOR ERE] Dig for a partial solution under an AND: "
          << m_andDig << "\n"
          << "c [CONSTRUCTOR ERE] Cut on MAX tree when low UB: "
          << m_cutUpperMax << "\n"
          << "c [CONSTRUCTOR ERE] Compute the connected component regarding "
             "the projected variables: "
          << m_cutUpperMax << "\n"
          << "c [CONSTRUCTOR ERE] Greedy init activated: "
          << m_greedyInitActivated << "\n";

    srand(0);

    // we create the SAT solver.
    OptionSolver optionSolver;
    optionSolver.solverName =
        SolverNameManager::getSolverName(vm["solver"].as<std::string>());
    m_solver = WrapperSolver::makeWrapperSolver(optionSolver, m_out);
    assert(m_solver);
    m_solver->initSolver(*m_problem);
    m_solver->setNeedModel(true);

    // we initialize the object that will give info about the problem.
    OptionSpecManager optSpecManager;
    optSpecManager.specUpdateType = SpecUpdateManager::getSpecUpdate(
        vm["occurrence-manager"].as<std::string>());
    m_specs = SpecManager::makeSpecManager(optSpecManager, *m_problem, m_out);

    // we initialize the object used to compute score and partition.
    OptionScoringMethod optionScoringMethod;
    optionScoringMethod.scoringMethodType =
        ScoringMethodTypeManager::getScoringMethodType(
            vm["scoring-method"].as<std::string>());
    m_hVar = ScoringMethod::makeScoringMethod(optionScoringMethod, *m_specs,
                                              *m_solver, m_out);
    m_hPhase =
        PhaseHeuristic::makePhaseHeuristic(vm, *m_specs, *m_solver, m_out);
    m_hPhaseInd = new PhaseHeuristicPolarity(*m_solver, true);

    // specify which variables are decisions, and which are not.
    m_redirectionPos.clear();
    m_isDecisionVariable.clear();
    m_isMaxDecisionVariable.clear();
    m_isProjectedVariable.clear();

    m_redirectionPos.resize(m_problem->getNbVar() + 1, 0);
    m_isDecisionVariable.resize(m_problem->getNbVar() + 1, false);
    m_isProjectedVariable.resize(m_problem->getNbVar() + 1, false);
    for (unsigned i = 0; i < m_problem->getIndVar().size(); i++) {
      Var v = m_problem->getIndVar()[i];
      m_isDecisionVariable[v] = true;
      m_redirectionPos[v] = i;
      m_isProjectedVariable[v] = true;
    }

    m_isMaxDecisionVariable.resize(m_problem->getNbVar() + 1, false);
    for (unsigned i = 0; i < m_problem->getMaxVar().size(); i++) {
      Var v = m_problem->getMaxVar()[i];
      m_isMaxDecisionVariable[v] = true;
      m_redirectionPos[v] = i;
    }

    // no partitioning heuristic for the moment.
    assert(m_hVar && m_hPhase);

    OptionCacheManager optionCacheManager;
    optionCacheManager.cachingMethod = CachingMehodManager::getCachingMethod(
        vm["cache-method"].as<std::string>()),

    optionCacheManager.optionCacheCleaningManager = {
        CacheCleaningStrategyManager::getCacheCleaningStrategy(
            vm["cache-reduction-strategy"].as<std::string>())};

    optionCacheManager.optionBucketManager = {
        ModeStoreManager::getModeStore(
            vm["cache-store-strategy"].as<std::string>()),
        ClauseRepresentationManager::getClauseRepresentation(
            vm["cache-clause-representation"].as<std::string>()),
        vm["cache-size-first-page"].as<unsigned long>(),
        vm["cache-size-additional-page"].as<unsigned long>(),
        vm["cache-clause-representation-combi-limitVar-sym"].as<unsigned>(),
        vm["cache-clause-representation-combi-limitVar-index"].as<unsigned>()};

    m_cacheInd = CacheManager<T>::makeCacheManager(
        optionCacheManager, m_problem->getNbVar(), m_specs, m_out);
    m_cacheMax = CacheManager<MaxSharpSatResult>::makeCacheManager(
        optionCacheManager, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    m_nbCallProj = m_nbDecisionNode = m_nbSplitMax = m_nbSplitInd =
        m_nbCallCall = 0;

    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);
    m_out << "c\n";

    // init the memory required for storing interpretation.
    m_memoryPages.push_back(new u_int8_t[c_sizePage]);
    m_posInMemoryPages = 0;
    m_sizeArray = m_problem->getMaxVar().size();

    // set the m_scale variable if needed.
    m_scale.valuation = getArray();
    for (unsigned i = 0; i < m_sizeArray; i++) m_scale.valuation[i] = 0;
    m_maxCount.valuation = getArray();
  }  // constructor

  /**
     Destructor.
   */
  ~ExistRandomExist() {
    delete m_problem;
    delete m_solver;
    delete m_specs;
    delete m_hVar;
    delete m_hPhase;
    delete m_cacheInd;
    delete m_cacheMax;

    for (auto page : m_memoryPages) delete[] page;
  }  // destructor

 private:
  /**
   * @brief Print out the solution.
   *
   * @param solution is the maxsharp SAT solution we want to print.
   */
  void printSolution(MaxSharpSatResult &solution, char status) {
    if (solution.count == T(0)) {
      std::cout << "s UNSAT\n";
      exit(0);
    }

    assert(solution.valuation);
    assert(m_problem->getMaxVar().size() == m_sizeArray);
    std::cout << "v ";
    for (unsigned i = 0; i < m_problem->getMaxVar().size(); i++) {
      std::cout << ((solution.valuation[i]) ? "" : "-")
                << m_problem->getMaxVar()[i] << " ";
    }
    std::cout << "0\n";
    std::cout << status << " " << std::fixed << std::setprecision(50)
              << solution.count << "\n";
  }  // printSolution

  /**
     Print out information about the solving process.

     @param[in] out, the stream we use to print out information.
  */
  inline void showInter(std::ostream &out) {
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCallCall << std::fixed
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCallProj << std::fixed
        << std::setprecision(2) << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << getTimer() << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cacheMax->getNbPositiveHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheMax->getNbNegativeHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cacheInd->getNbPositiveHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->getNbNegativeHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplitMax << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplitInd << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCutUpperBoundMaxPart << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCutUpperBoundIndPart << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbPureMax << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbPureInd << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->usedMemory() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbDecisionNode << "|"
        << std::scientific << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_maxCount.count << "|\n";
  }  // showInter

  /**
     Print out a line of dashes.

     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out) {
    out << "c ";
    for (int i = 0; i < NB_SEP; i++) out << "-";
    out << "\n";
  }  // separator

  /**
     Print out the header information.

     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out) {
    separator(out);
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(m)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(i)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit(m)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit(m)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit(i)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit(i)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split(m)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split(i)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#cutUb(m)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#cutUb(i)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#pure(m)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#pure(i)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "max#count"
        << "|\n";
    separator(out);
  }  // showHeader

  /**
     Print out information when it is required.

     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out) {
    unsigned nbCall = m_nbCallCall + m_nbCallProj;
    if (!(nbCall & (MASK_HEADER))) showHeader(out);
    if (nbCall && !(nbCall & MASK_SHOWRUN_MC)) showInter(out);
  }  // showRun

  /**
     Print out the final stat.

     @param[in] out, the stream we use to print out information.
   */
  inline void printFinalStats(std::ostream &out) {
    separator(out);
    out << "c\n"
        << "c \033[1m\033[31mStatistics \033[0m\n"
        << "c \033[33mCompilation Information\033[0m\n"
        << "c Number of recursive call: " << m_nbCallCall << "\n"
        << "c Number of split formula (max): " << m_nbSplitMax << "\n"
        << "c Number of split formula (ind): " << m_nbSplitInd << "\n"
        << "c Number of units because pure (max): " << m_nbPureMax << "\n"
        << "c Number of units because pure (ind): " << m_nbPureInd << "\n"
        << "c Number of cut because upper bound (max): "
        << m_nbCutUpperBoundMaxPart << "\n"
        << "c Number of cut because upper bound (ind): "
        << m_nbCutUpperBoundIndPart << "\n"
        << "c Number of decision: " << m_nbDecisionNode << "\n"
        << "c\n";
    m_cacheInd->printCacheInformation(out);
    out << "c\n";
    m_cacheMax->printCacheInformation(out);
    out << "c Final time: " << getTimer() << "\n";
    out << "c\n";
  }  // printFinalStat

  /**
   * @brief Get a pointer on an allocated array of size m_sizeArray (which is
   * set once in the constructor).
   *
   * @return a pointer on a u_int8_t array.
   */
  u_int8_t *getArray() {
    u_int8_t *ret = &(m_memoryPages.back()[m_posInMemoryPages]);
    m_posInMemoryPages += m_sizeArray;
    if (m_posInMemoryPages > c_sizePage) {
      m_memoryPages.push_back(new u_int8_t[c_sizePage]);
      m_posInMemoryPages = 0;
      ret = m_memoryPages.back();
    }
    return ret;
  }  // getArray

  /**
   * Expel from a set of variables the ones they are marked as being decidable.
   * @param[out] vars, the set of variables we search to filter.
   * @param[in] isDecisionvariable, a type decision vector that marks as true
   * decision variables.
   */
  void expelNoDecisionVar(std::vector<Var> &vars) {
    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++)
      if (m_isDecisionVariable[vars[i]]) vars[j++] = vars[i];
    vars.resize(j);
  }  // expelNoDecisionVar

  /**
     Expel from a set of variables the ones they are marked as being decidable.

     @param[out] lits, the set of literals we search to filter.

     @param[in] isDecisionvariable, a boolean vector that marks as true decision
     variables.
   */
  void expelNoDecisionLit(std::vector<Lit> &lits) {
    unsigned j = 0;
    for (unsigned i = 0; i < lits.size(); i++)
      if (m_isDecisionVariable[lits[i].var()]) lits[j++] = lits[i];
    lits.resize(j);
  }  // expelNoDecisionLit

  /**
   * @brief Estimate the maximum number of models we can get when considering
   * the given set of variables.
   *
   * @param setOfVar is the considered set of variables.
   * @return T is the resulting number of models (upper bound).
   */
  T computeUpper(std::vector<Var> &setOfVar) {
    T ret = 1;
    for (auto v : setOfVar)
      if (m_isProjectedVariable[v]) ret = ret * 2;
    return ret;
  }  // computeUpper

  /**
   * @brief Apply an or logic between resValuation and orValuation (the result
   * is stored in resValuation).
   *
   * @param vars is the set of variables we are considering for the OR.
   * @param[out] resValuation is a 'boolean' vector that also receive the
   * result.
   * @param orValuation is another 'boolean' vector used for the OR
   */
  void orOnMaxVar(std::vector<Var> &vars, u_int8_t *resValuation,
                  u_int8_t *orValuation) {
    for (auto v : vars) {
      if (m_isMaxDecisionVariable[v])
        resValuation[m_redirectionPos[v]] |= orValuation[m_redirectionPos[v]];
    }
  }  // disjunctionOnMaxVariable

  /**
   * @brief Update the bound if needed.
   *
   * @param result is a solution we found.
   * @param vars is the set of variables of the component that is considered to
   * compute result.
   */
  void updateBound(MaxSharpSatResult &result, std::vector<Var> &vars) {
    if (!m_isUnderAnd && result.count * m_scale.count > m_maxCount.count) {
      m_maxCount.count = result.count * m_scale.count;

      assert(result.valuation);
      for (unsigned i = 0; i < m_sizeArray; i++)
        m_maxCount.valuation[i] = m_scale.valuation[i];

      for (auto v : vars)
        if (m_isMaxDecisionVariable[v])
          m_maxCount.valuation[m_redirectionPos[v]] =
              result.valuation[m_redirectionPos[v]];

      if (m_threshold < 0 || m_maxCount.count < T(m_threshold)) {
        m_out << "i " << ++m_countUpdateMaxCount << " " << std::fixed
              << std::setprecision(2) << getTimer() << " ";
        m_out << std::fixed << std::setprecision(50) << m_maxCount.count
              << "\n";
      } else {
        m_out << "c Stop because we found out a good enough solution\n";
        m_out << "r SATISFIABLE\n";
        printSolution(m_maxCount, 's');
        m_stopProcess = true;
      }
    }
  }  // updateBound

  /**
   * @brief Compute the connected component.
   *
   * @param[out] varCo is the list of connected component.
   * @param setOfVar is the current set of variables.
   * @param[out] freeVar is the set of free variables.
   * @return the number of component.
   */
  int computeConnectedComponent(std::vector<std::vector<Var>> &varCo,
                                std::vector<Var> &setOfVar,
                                std::vector<Var> &freeVar) {
    if (m_componentOnProjected)
      return m_specs->computeConnectedComponentTargeted(
          varCo, setOfVar, m_isProjectedVariable, freeVar);

    return m_specs->computeConnectedComponent(varCo, setOfVar, freeVar);
  }  // computeConnectedComponent

  /**
   * @brief Given the selected heuristic, return the way we want to assign the
   * given variable (actually we want the sign).
   *
   * @param v is the variable we want to assign.
   * @return 1 if we want to assign to false, 0 otherwise/
   */
  inline bool selectPhase(Var v) {
    assert(m_isMaxDecisionVariable[v]);
    int rdm = rand() % 100;
    if (rdm <= m_heuristicMaxRdm) return rdm & 1;

    if (m_scale.count > 0 && m_heuristicMax == "best")
      return m_scale.valuation[m_redirectionPos[v]];
    return m_hPhase->selectPhase(v);
  }  // selectPhase

  /**
   * @brief Dig for pure literals we can assign.
   *
   * @param setOfVar, the current set of considered variables
   * @param[out] unitsLit is the place where the new literals assigned will be
   * pushed.
   */
  void assignPureLiteral(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
                         unsigned &countPure) {
    std::vector<Lit> pureLit;
    for (auto &v : setOfVar) {
      if (m_isProjectedVariable[v]) continue;
      if (m_specs->varIsAssigned(v)) continue;

      Lit l = Lit::makeLitTrue(v);
      if (!m_specs->getNbOccurrence(l) && m_specs->getNbOccurrence(~l))
        pureLit.push_back(~l);
      if (!m_specs->getNbOccurrence(~l) && m_specs->getNbOccurrence(l))
        pureLit.push_back(l);
    }
    if (pureLit.size()) {
      for (auto &l : pureLit) unitsLit.push_back(l);
      m_specs->preUpdate(pureLit);
      countPure += pureLit.size();
    }
  }  // assignPureLiteral

  /**
   * @brief Search for a valuation of the max variables that maximizes the
   * number of models on the remaning formula where some variables are
   * forget.
   *
   * @param setOfVar, the current set of considered variables.
   * @param unitsLit, the set of unit literal detected at this level.
   * @param freeVariable, the variables which become free decision node.
   * @param out, the stream we use to print out logs.
   * @param result, the structure where is stored the result.
   */
  void searchMaxValuation(std::vector<Var> &setOfVar,
                          std::vector<Lit> &unitsLit,
                          std::vector<Var> &freeVariable, std::ostream &out,
                          MaxSharpSatResult &result, T ifTaut) {
    assert(!m_hasBeenStop);
    if (m_stopProcess) return;

    showRun(out);
    m_nbCallCall++;

    // is the problem still satisfiable?
    if (!m_solver->solve(setOfVar)) {
      result = {T(0), NULL};
      return;
    }

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);
    assignPureLiteral(setOfVar, unitsLit, m_nbPureMax);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    int nbComponent =
        computeConnectedComponent(varConnected, setOfVar, freeVariable);

    // init the returned result.
    result.valuation = getArray();
    for (unsigned i = 0; i < m_sizeArray; i++) result.valuation[i] = 0;

    // set a valuation for fixed variables.
    T saveCount = m_scale.count, fixInd = T(1), mustMultiply = T(1);

    for (auto &v : freeVariable)
      if (m_isMaxDecisionVariable[v]) {
        Lit l = Lit::makeLit(v, m_solver->getModelVar(v) == l_False);
        m_scale.valuation[m_redirectionPos[v]] = 1 - l.sign();
        result.valuation[m_redirectionPos[v]] = 1 - l.sign();
      } else if (m_isDecisionVariable[v])
        fixInd *= T(m_problem->getWeightVar(v));

    // consider the unit literals that belong to max
    for (auto &l : unitsLit)
      if (m_isMaxDecisionVariable[l.var()]) {
        m_scale.valuation[m_redirectionPos[l.var()]] = 1 - l.sign();
        result.valuation[m_redirectionPos[l.var()]] = 1 - l.sign();
      } else if (m_isDecisionVariable[l.var()]) {
        std::cout << "Propagate one literal from the ind " << l.human() << "\n";
        exit(0);
        fixInd *= T(m_problem->getWeightLit(l));
      }

    if (m_cutUpperMax && ifTaut * fixInd <= m_maxCount.count) {
      result.count = T(0);
      m_nbCutUpperBoundMaxPart++;
      m_specs->postUpdate(unitsLit);
      return;
    }

    result.count = T(1);
    m_scale.count = m_scale.count * fixInd;

    expelNoDecisionVar(freeVariable);
    bool wasUnderAnd = m_isUnderAnd;

    // dig for an assignment for each component (execpt the first one).
    std::vector<MaxSharpSatResult> andCount;
    if (!m_andDig)
      m_isUnderAnd = wasUnderAnd || nbComponent > 1;
    else {
      for (int cp = 1; cp < nbComponent; cp++) {
        andCount.push_back({T(0), NULL});
        greedySearch(varConnected[cp], out, andCount.back());
        orOnMaxVar(varConnected[cp], m_scale.valuation,
                   andCount.back().valuation);
        assert(andCount.back().count != T(0));
        m_scale.count *= andCount.back().count;
      }
    }

    // consider each connected component.
    if (nbComponent) {
      m_nbSplitMax += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        TmpEntry<MaxSharpSatResult> cb = m_cacheMax->searchInCache(connected);

        // should divide if we are under an AND and if we manage the option.
        if (m_andDig && cp > 0)
          m_scale.count = m_scale.count / andCount[cp - 1].count;

        if (cb.defined) {
          mustMultiply = cb.getValue().count;
          if (cb.getValue().valuation)
            orOnMaxVar(connected, result.valuation, cb.getValue().valuation);
        } else {
          MaxSharpSatResult tmpResult;
          searchMaxSharpSatDecision(connected, out, tmpResult, ifTaut * fixInd);

          if (!m_hasBeenStop)
            m_cacheMax->addInCache(cb, tmpResult);
          else
            m_cacheMax->releaseMemory(cb.getCachedBucket());

          mustMultiply = tmpResult.count;
          if (tmpResult.valuation)
            orOnMaxVar(connected, result.valuation, tmpResult.valuation);
        }

        result.count = result.count * mustMultiply;

        // should multiply if we are under an AND and if we manage the option.
        if (nbComponent > 1 && m_andDig) {
          m_scale.count = m_scale.count * mustMultiply;

          for (auto &v : connected)
            if (m_isMaxDecisionVariable[v])
              m_scale.valuation[m_redirectionPos[v]] =
                  result.valuation[m_redirectionPos[v]];
        }
      }
    }  // else we have a tautology

    m_isUnderAnd = wasUnderAnd;
    m_scale.count = saveCount;

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit);

    // update the global maxcount if needed.
    m_scale.count *= fixInd;
    updateBound(result, setOfVar);
    m_scale.count = saveCount;
  }  // searchMaxValuation

  /**
   * This function select a variable and compile a decision node.
   *
   * @param[in] connected, the set of variable present in the current
   * problem.
   * @param[in] out, the stream we use to print out logs.
   * @param[out] result, the best solution found.
   */
  void searchMaxSharpSatDecision(std::vector<Var> &connected, std::ostream &out,
                                 MaxSharpSatResult &result, T ifTaut) {
    if (m_stopProcess) return;

    // search the next variable to branch on
    Var v =
        m_hVar->selectVariable(connected, *m_specs, m_isMaxDecisionVariable);

    if (v == var_Undef) {
      std::vector<Lit> unitsLit;
      std::vector<Var> freeVar;
      m_hasBeenStop = false;

      exit(0);

      std::cout << "Variables: ";
      for (auto &v : connected) {
        if (m_isProjectedVariable[v]) std::cout << v << " ";
      }
      std::cout << " ---> " << m_maxCount.count / ifTaut << "\n";

      result.count = countInd_(connected, unitsLit, freeVar, out,
                               m_maxCount.count / ifTaut);
      result.count *= m_problem->computeWeightUnitFree<T>(unitsLit, freeVar);
      result.valuation = NULL;
      m_hasBeenStop = false;

      std::cout << " result = " << result.count << "\n\n";
      return;
    }

    Lit l = Lit::makeLit(v, selectPhase(v));
    m_nbDecisionNode++;

    std::cout << "Max Decision : " << l << " 0\n";

    // consider the two value for l
    DataBranch<T> b[2];
    MaxSharpSatResult res[2];

    // search max#sat for the first phase.
    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    searchMaxValuation(connected, b[0].unitLits, b[0].freeVars, out, res[0],
                       ifTaut);

    m_solver->popAssumption();
    b[0].d = res[0].count * m_problem->computeWeightUnitFree<T>(b[0]);

    // search max#sat for the next phase.
    if (m_solver->isInAssumption(l))
      res[1].count = T(0);
    else if (m_solver->isInAssumption(~l))
      searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out, res[1],
                         ifTaut);
    else {
      m_solver->pushAssumption(~l);
      searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out, res[1],
                         ifTaut);
      m_solver->popAssumption();
    }

    b[1].d = res[1].count * m_problem->computeWeightUnitFree<T>(b[1]);

    // aggregation with max.
    result.count = (b[0].d > b[1].d) ? b[0].d : b[1].d;
    result.valuation = (b[0].d > b[1].d) ? res[0].valuation : res[1].valuation;
  }  // searchMaxSharpSatDecision

  /**
   * @brief Count the number of projected models.
   *
   * @param setOfVar, the current set of considered variables
   * @param unitsLit, the set of unit literal detected at this level
   * @param freeVariable, the variables which become free decision node
   * @param out, the stream we use to print out logs.
   *
   * \return the number of models.
   */
  T countInd_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
              std::vector<Var> &freeVariable, std::ostream &out, T targetMin) {
    if (m_hasBeenStop) return T(0);
    if (m_stopProcess) return T(0);
#if TEST
    std::cout << "countInd: " << targetMin << "\n";
#endif
    if (targetMin >= T(1)) {
      m_hasBeenStop = true;
      m_nbCutUpperBoundIndPart++;
    }

    showRun(out);
    m_nbCallProj++;

    if (!m_solver->solve(setOfVar)) {
      std::cout << "unsat\n";
      m_solver->getCore();
      return T(0);
    }

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);

    T fixInd = T(1);
    for (auto &l : unitsLit)
      if (m_isProjectedVariable[l.var()]) {
        if (!m_solver->isInAssumption(l.var()))
          std::cout << "Propagate: " << l.human() << " "
                    << m_problem->getWeightLit(l) << "\n";

        m_solver->getLastIUP(l);
        fixInd *= T(m_problem->getWeightLit(l));
      }

#if TEST
    std::cout << "fixInd = " << fixInd << " -> " << targetMin << "\n";
#endif
    assignPureLiteral(setOfVar, unitsLit, m_nbPureInd);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    int nbComponent =
        computeConnectedComponent(varConnected, setOfVar, freeVariable);
    expelNoDecisionVar(freeVariable);

    // consider each connected component.
    T result = T(1);
    if (nbComponent) {
      m_nbSplitInd += (nbComponent > 1) ? nbComponent : 0;

      for (int cp = 0; !m_hasBeenStop && cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];
        TmpEntry<T> cb = m_cacheInd->searchInCache(connected);

        if (cb.defined) {
          std::cout << "Hit positive\n";
          result = result * cb.getValue();
        } else {
          T curr = countIndDecisionNode(connected, out,
                                        (targetMin / result) / fixInd);
          if (!m_hasBeenStop)
            m_cacheInd->addInCache(cb, curr);
          else
            m_cacheInd->releaseMemory(cb.getCachedBucket());
          result = result * curr;
        }

        if (!m_hasBeenStop && result < targetMin) {
          m_hasBeenStop = true;
          m_nbCutUpperBoundIndPart++;
        }
      }
    }  // else we have a tautology

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit);
#if TEST
    std::cout << "result " << result << " ---> ";
    for (auto &l : m_solver->getAssumption())
      if (m_isProjectedVariable[l.var()]) std::cout << l << " ";
    std::cout << "\n";
#endif
    return result;
  }  // countInd_

  /**
   * @brief This function select a variable and compile a decision node.
   *
   * @param connected, the set of variable present in the current
   * problem.
   * @param out, the stream we use to print out logs.
   * \return the number of computed models.
   */
  T countIndDecisionNode(std::vector<Var> &connected, std::ostream &out,
                         T targetMin) {
    if (targetMin >= T(1)) {
      m_hasBeenStop = true;
      m_nbCutUpperBoundIndPart++;
      return T(0);
    }
    if (m_stopProcess) return T(0);

    static int counterCall = 0;
    int currentCall = ++counterCall;

    if (currentCall == 10310) exit(0);

    // search the next variable to branch on
    Var v = m_hVar->selectVariable(connected, *m_specs, m_isDecisionVariable);
    if (v == var_Undef) {
#if TEST
      std::cout << currentCall << " Taut\n";
#endif
      return T(1);
    }

    // select a variable for decision.
    Lit l = Lit::makeLit(v, m_hPhaseInd->selectPhase(v));
    m_nbDecisionNode++;

    std::cout << currentCall << " Target Min = " << targetMin << "\n";
    std::cout << currentCall << " Decision up:" << l.human() << "\n";

    // consider the two value for l
    DataBranch<T> b[2];

    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    b[0].d = countInd_(connected, b[0].unitLits, b[0].freeVars, out,
                       targetMin - T(m_problem->getWeightLit(~l)));
    m_solver->popAssumption();

    // compute the next lower regarding the already compute information.
    b[0].d *= m_problem->computeWeightUnitFree<T>(b[0]);

    std::cout << currentCall << " result from decision " << l.human() << " = "
              << b[0].d << "\n";
    std::cout << currentCall << " Decision back: " << (~l).human() << "\n";

    if (m_solver->isInAssumption(l)) {
      std::cout << "unsat on the left\n";
      b[1].d = 0;
    } else if (m_solver->isInAssumption(~l))
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out,
                         targetMin - b[0].d);
    else {
      m_solver->pushAssumption(~l);
      b[1].d = countInd_(connected, b[1].unitLits, b[1].freeVars, out,
                         targetMin - b[0].d);
      m_solver->popAssumption();
    }

    b[1].d *= m_problem->computeWeightUnitFree<T>(b[1]);

    std::cout << currentCall << " result from decision " << (~l).human()
              << " = " << b[1].d << "\n";

    return b[0].d + b[1].d;
  }  // computeDecisionNode

  /**
   * @brief Search for an interpretation that maximize the number of models in
   * a greedy search.
   *
   * @param setOfVar is the set of variables we are considering.
   * @param out is the stream where is printed out the logs.
   * @param result is the interpretation and the related number of models.
   */
  void greedySearch(std::vector<Var> &setOfVar, std::ostream &out,
                    MaxSharpSatResult &result, bool runOnce = true) {
    assert(!m_hasBeenStop);

    // first: search for a model to init the interpretation.
    if (!m_solver->solve(setOfVar)) {
      result.count = T(0);
      result.valuation = NULL;
      return;
    }

    // collect the model.
    result.valuation = getArray();
    result.count = T(0);

    // create the assumption and get the variables for the couting process.
    std::vector<Lit> unitsAssums;
    std::vector<Var> vars = setOfVar;
    unsigned j = 0;
    for (unsigned i = 0; i < vars.size(); i++) {
      if (m_isMaxDecisionVariable[vars[i]])
        unitsAssums.push_back(
            Lit::makeLit(vars[i], m_solver->getModelVar(vars[i]) == l_False));
      else
        vars[j++] = vars[i];
    }
    vars.resize(j);

    // explore around the current assumption.
    std::vector<Lit> negTest = unitsAssums;
    for (auto &l : negTest) l = ~l;
    std::random_shuffle(negTest.begin(), negTest.end());

    bool first = true;
    while (!m_stopProcess && (first || negTest.size())) {
      first = false;
      if (!runOnce)
        m_out << "c [ERE] Greedy: " << negTest.size() << " " << result.count
              << " \n";
      for (auto &l : unitsAssums) m_solver->pushAssumption(l);
      m_solver->propagateAssumption();

      std::vector<Lit> unitsLit;
      std::vector<Var> freeVariable;
      m_specs->preUpdate(unitsAssums);

      T tmpCount = countInd_(vars, unitsLit, freeVariable, out, T(0));
      if (!m_stopProcess) {
        assert(tmpCount != T(0));
        tmpCount = tmpCount *
                   m_problem->computeWeightUnitFree<T>(unitsLit, freeVariable);

        if (tmpCount > result.count) {
          result.count = tmpCount;
          for (auto &l : unitsAssums)
            result.valuation[m_redirectionPos[l.var()]] = 1 - l.sign();
        }
      }

      m_solver->popAssumption(unitsAssums.size());
      m_specs->postUpdate(unitsAssums);

      if (runOnce) break;

      // update for the next round.
      bool isSat = false;
      while (!isSat && negTest.size()) {
        Lit l = negTest.back();
        negTest.pop_back();

        m_solver->pushAssumption(l);
        isSat = m_solver->solve(setOfVar);
        if (isSat) {
          // get the next assumption.
          for (auto &l : unitsAssums)
            l = Lit::makeLit(l.var(),
                             m_solver->getModelVar(l.var()) == l_False);

          j = 0;
          for (unsigned i = 0; i < negTest.size(); i++)
            if ((m_solver->getModelVar(negTest[i].var()) == l_False) !=
                l.sign())
              negTest[j++] = negTest[i];
          negTest.resize(j);
        }
        m_solver->popAssumption();
      }
    }
  }  // greedySearch

  /**
   * @brief Search an assignation of Max variables that maximize the number of
   * models of the problem conditionned by this assignation.
   *
   * @param setOfVar is the set of variables of the considered problem.
   * @param out is the stream where are printed out the logs.
   * @param[out] result is the place where is stored the result.
   * @param warmStart is an option to activate/deactivate the warm start
   * strategy (by defaut it is deactivate).
   */
  void compute(std::vector<Var> &setOfVar, std::ostream &out,
               MaxSharpSatResult &result, bool warmStart = true) {
    if (m_problem->isUnsat() ||
        (warmStart && !m_panicMode &&
         !m_solver->warmStart(29, 11, setOfVar, m_out))) {
      result = {T(0), NULL};
      return;
    }

    if (m_greedyInitActivated) {
      MaxSharpSatResult greedyResult;
      greedySearch(setOfVar, out, greedyResult, false);
      updateBound(greedyResult, setOfVar);
      std::cout << "c Greedy search done: " << greedyResult.count << "\n";
    }

    // add as  (negated) unit literal with a weight of 0.
    unsigned nbZero = 0;
    for (auto v : setOfVar) {
      Lit l = Lit::makeLit(v, false);

      if (m_problem->getWeightLit(l) == 0) {
        nbZero++;
        m_solver->pushAssumption(~l);
      }
      if (m_problem->getWeightLit(~l) == 0) {
        nbZero++;
        m_solver->pushAssumption(l);
      }
    }

    std::cout << "c Number of zero weight literals: " << nbZero << "\n";

    DataBranch<T> b;
    searchMaxValuation(setOfVar, b.unitLits, b.freeVars, out, result, T(1));
    assert(result.valuation);
    if (!m_stopProcess)
      result.count *=
          m_problem->computeWeightUnitFree<T>(b.unitLits, b.freeVars);

    if (nbZero) m_solver->popAssumption(nbZero);
  }  // compute

 public:
  /**
   * @brief Stop the current search and print out the best solution found so
   * far.
   *
   */
  void interrupt() override {
    if (m_maxCount.count < 0)
      std::cout << "No solution found so far\n";
    else {
      std::cout << "Processus interrupted, here is the best solution found\n";
      printSolution(m_maxCount, 'k');
    }
  }  // interrupt

  /**
   * @brief Search for the instantiation of the variables of
   * m_problem->getMaxVar() that maximize the number of the remaining
   * variables where the variables not belonging to m_problem->getIndVar()
   * are existantially quantified.
   *
   * @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm) {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);

    MaxSharpSatResult result;
    compute(setOfVar, m_out, result);
    printFinalStats(m_out);
    if (!m_stopProcess) {
      if (m_threshold < 0)
        printSolution(m_maxCount, 'o');
      else {
        if (result.count >= T(m_threshold)) {
          std::cout << "r SATISFIABLE\n";
          printSolution(result, 's');
        } else {
          std::cout << "r UNSATISFIABLE\n";
          printSolution(result, 'o');
        }
      }
    }
  }  // run

};  // end class
}  // namespace d4
