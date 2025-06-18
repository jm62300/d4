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

#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <ios>
#include <iostream>
#include <string>
#include <vector>

#include "Counter.hpp"
#include "DataBranch.hpp"
#include "MethodManager.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/heuristics/partialOrder/PartialOrderHeuristic.hpp"
#include "src/heuristics/phaseSelection/PhaseHeuristic.hpp"
#include "src/heuristics/scoringVariable/ScoringMethod.hpp"
#include "src/methods/nnf/Node.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/options/methods/OptionMaxTMethod.hpp"
#include "src/options/solvers/OptionSolver.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/utils/MemoryStat.hpp"

namespace d4 {

template <class T>
class Counter;

template <class T, class A>
class MaxT : public MethodManager {
  enum TypeDecision { NO_DEC, EXIST_DEC, MAX_DEC };

  struct MaxSharpSatResult {
    T count;
    u_int8_t *valuation;

    MaxSharpSatResult() : count(T()), valuation(NULL) {}
    MaxSharpSatResult(const T c, u_int8_t *v) : count(c), valuation(v) {}

    void display(unsigned size) {
      assert(valuation);
      for (unsigned i = 0; i < size; i++)
        std::cout << (int)getBit(valuation, i) << " ";
      std::cout << "\n";
    }
  };

 private:
  const unsigned NB_SEP = 195;

  bool m_optThreshold = false;
  bool m_solutionFound = false;
  unsigned m_nbCallCall;
  unsigned m_nbCallProj;
  unsigned m_nbSplitMax = 0;
  unsigned m_nbSplitInd = 0;
  unsigned m_nbDecisionNode;
  unsigned m_optCachedMax;
  unsigned m_optCachedInd;
  unsigned m_stampIdx;

  bool m_isUnderAnd = false;
  bool m_greedyInitActivated;

  std::vector<unsigned> m_stampVar;
  std::vector<std::vector<Lit>> clauses;

  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_isProjectedVariable;
  std::vector<bool> m_isMaxDecisionVariable;
  std::vector<unsigned> m_redirectionPos;
  unsigned m_countUpdateMaxCount = 0;

  const unsigned c_sizePage = 1 << 25;
  std::vector<u_int8_t *> m_memoryPages;
  unsigned m_posInMemoryPages;
  unsigned m_sizeArray;

  ProblemManager *m_problem;
  WrapperSolver *m_solver;
  FormulaManager *m_specs;
  ScoringMethod *m_hVarMax;
  PhaseHeuristic *m_hPhaseMax;

  BranchingHeuristic *m_heuristicInd;

  CacheManager<T> *m_cacheInd;
  CacheManager<MaxSharpSatResult> *m_cacheMax;

  std::ostream m_out;
  TmpEntry<MaxSharpSatResult> NULL_CACHE_ENTRY;
  bool m_cacheMaxActivated = true;
  TmpEntry<T> NULL_CACHE_ENTRY_T;
  bool m_cacheIndActivated = true;

  bool m_stopProcess = false;
  std::string m_heuristicMax = "none";
  unsigned m_heuristicMaxRdm = 0;

  MaxSharpSatResult m_scale;
  MaxSharpSatResult m_maxCount;
  double m_accTime;

  A m_aggregator;
  T m_threshold;

 public:
  /**
   * @brief Construct a new max#sat solver.
   *
   * @param options are the option.
   * @param initProblem is the problem we deal with.
   * @param out is the stream where are printed out the logs.
   */
  MaxT(const OptionMaxTMethod &options, ProblemManager *initProblem,
       std::ostream &out)
      : m_problem(initProblem), m_out(nullptr), m_aggregator(initProblem) {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());

    m_scale = {m_aggregator.mulIdentity(), NULL};
    m_maxCount = {m_aggregator.min(), NULL};

    m_heuristicMax = options.phaseHeuristicMax;
    m_heuristicMaxRdm = options.randomPhaseHeuristicMax;
    m_greedyInitActivated = options.greedyInitActivated;
    out << "c " << options << '\n';

    m_cacheMaxActivated = options.optionCacheManagerMax.isActivated;
    m_cacheIndActivated = options.optionCacheManagerInd.isActivated;

    // we create the SAT solver.
    m_solver = WrapperSolver::makeWrapperSolver(options.optionSolver,
                                                *m_problem, m_out);
    assert(m_solver);
    m_solver->initSolver(*m_problem);
    m_solver->setNeedModel(true);

    // we initialize the object that will give info about the problem.
    m_specs = FormulaManager::makeFormulaManager(options.optionSpecManager,
                                                 *m_problem, m_out);
    assert(m_specs);

    // we initialize the object used to compute score and partition.
    m_hVarMax = ScoringMethod::makeScoringMethod(
        options.optionBranchingHeuristicMax, *m_specs, *m_solver, m_out);
    m_hPhaseMax = PhaseHeuristic::makePhaseHeuristic(
        options.optionBranchingHeuristicMax, *m_specs, *m_solver, m_out);

    m_heuristicInd = BranchingHeuristic::makeBranchingHeuristic(
        options.optionBranchingHeuristicInd, m_problem, m_specs, *m_solver,
        *m_solver, m_out);

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
    m_cacheInd = CacheManager<T>::makeCacheManager(
        options.optionCacheManagerInd, m_problem->getNbVar(), m_specs, m_out);
    m_cacheMax = CacheManager<MaxSharpSatResult>::makeCacheManager(
        options.optionCacheManagerMax, m_problem->getNbVar(), m_specs, m_out);

    // init the clock time.
    initTimer();

    m_optCachedMax = options.optionCacheManagerMax.isActivated;
    m_optCachedInd = options.optionCacheManagerInd.isActivated;
    m_nbCallProj = m_nbDecisionNode = m_nbSplitMax = m_nbSplitInd =
        m_nbCallCall = 0;

    m_stampIdx = 0;
    m_stampVar.resize(m_specs->getNbVariable() + 1, 0);
    m_out << "c\n";

    // init the memory required for storing interpretation.
    m_memoryPages.push_back(new u_int8_t[c_sizePage]);
    m_posInMemoryPages = 0;
    m_sizeArray = computeSizeArray(m_problem->getMaxVar().size());

    // set the m_scale variable if needed.
    out << "c [MAXT] The size of the valuation array is: " << m_sizeArray
        << '\n';
    m_scale.valuation = getArray();
    setZeroArray(m_scale.valuation);
    m_maxCount.valuation = getArray();

    m_optThreshold = options.thresholdList.size();
    if (m_optThreshold) {
      m_threshold = T(options.thresholdList);
      out << "c [MAXT] Set the threshold limit: " << m_threshold << '\n';
    }

    m_accTime = 0;
  }  // constructor

  /**
     Destructor.
   */
  ~MaxT() {
    delete m_problem;
    delete m_solver;
    delete m_specs;
    delete m_hVarMax;
    delete m_hPhaseMax;

    delete m_heuristicInd;

    delete m_cacheInd;
    delete m_cacheMax;

    for (auto page : m_memoryPages) delete[] page;
  }  // destructor

 private:
  /**
   * @brief Initializes the given array by setting all elements to zero.
   *
   * This function fills the provided array with zero values, ensuring
   * it is properly initialized for further use.
   *
   * @param[out] arr Pointer to the array that needs to be zero-initialized.
   */
  inline void setZeroArray(uint8_t *arr) {
    std::memset(arr, 0, m_sizeArray);
  }  // setZeroArray

  inline unsigned computeSizeArray(unsigned maxVal) {
    return 1 + (maxVal >> 3);
  }

  inline u_int8_t getBit(uint8_t *arr, int idx) {
    return (arr[idx >> 3] >> (idx & 7)) & 1;
  }  // getBit

  inline void setBit(uint8_t *arr, int idx, u_int8_t val) {
    if (val)
      arr[idx >> 3] |= 1 << (idx & 7);
    else
      arr[idx >> 3] &= ~(1 << (idx & 7));
  }  // setBit

  inline void orBit(uint8_t *dst, const uint8_t *src, unsigned idx) {
    dst[idx >> 3] |= src[idx >> 3] & (1 << (idx & 7));
  }  // orBit

  /**
   * @brief Copies the contents of the source array into the destination array.
   *
   * This function performs an element-wise copy from the source array to the
   * destination array, ensuring that all values are transferred correctly.
   *
   * @param[out] dst Pointer to the destination array where data will be copied.
   * @param[in] src Pointer to the source array from which data is copied.
   */
  inline void setArr(uint8_t *dst, const uint8_t *src) {
    std::memcpy(dst, src, m_sizeArray);
  }  // setArr

  /**
   * @brief Print out the solution.
   *
   * @param solution is the maxsharp SAT solution we want to print.
   */
  void printSolution(MaxSharpSatResult &solution, char status) {
    if (!m_solutionFound) {
      std::cout << "s UNSAT\n";
      exit(0);
    }

    assert(solution.valuation);
    std::cout << "v ";
    for (unsigned i = 0; i < m_problem->getMaxVar().size(); i++) {
      std::cout << ((getBit(solution.valuation, i)) ? "" : "-")
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
    out << "c " << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCallCall
        << std::fixed << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbCallProj
        << std::fixed << std::setprecision(2) << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << getTimer() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_accTime << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheMax->getNbPositiveHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cacheMax->getNbNegativeHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->getNbPositiveHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC)
        << m_cacheInd->getNbNegativeHit() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplitMax << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbSplitInd << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cacheInd->usedMemory() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak() << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << m_nbDecisionNode << "|"
        << std::scientific << std::setw(WIDTH_PRINT_COLUMN_MC * 2)
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
    out << "c " << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(m)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#call(i)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "time" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "timeInd" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit(m)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit(m)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit(i)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit(i)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split(m)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split(i)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node" << "|"
        << std::setw(WIDTH_PRINT_COLUMN_MC * 2) << "max#count" << "|\n";
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
        << "c Number of decision: " << m_nbDecisionNode << "\n"
        << "c\n";
    m_cacheInd->printCacheInformation(out);
    out << "c\n";
    m_cacheMax->printCacheInformation(out);
    out << "c Final time: " << getTimer() << '\n';
    out << "c Final time Ind: " << m_accTime << '\n';
    out << "c\n";
  }  // printFinalStat

  /**
   * @brief Get a pointer on an allocated array of size m_sizeArray (which is
   * set once in the constructor).
   *
   * @return a pointer on a u_int8_t array.
   */
  inline u_int8_t *getArray() {
    u_int8_t *ret = &(m_memoryPages.back()[m_posInMemoryPages]);
    m_posInMemoryPages += m_sizeArray;
    if (m_posInMemoryPages > c_sizePage) {
      m_out << "c [MAXT] Allocate memory for valuation: "
            << m_memoryPages.size() * c_sizePage << '\n';
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
        orBit(resValuation, orValuation, m_redirectionPos[v]);
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
    if (!m_isUnderAnd &&
        (!m_solutionFound || result.count * m_scale.count > m_maxCount.count)) {
      m_maxCount.count = result.count * m_scale.count;

      assert(result.valuation);
      setArr(m_maxCount.valuation, m_scale.valuation);

      for (auto v : vars)
        if (m_isMaxDecisionVariable[v]) {
          setBit(m_maxCount.valuation, m_redirectionPos[v],
                 getBit(result.valuation, m_redirectionPos[v]));

          assert(getBit(result.valuation, m_redirectionPos[v]) ==
                 getBit(m_maxCount.valuation, m_redirectionPos[v]));
        }

      m_solutionFound = true;

      if (m_optThreshold && m_threshold <= m_maxCount.count) {
        std::cout << "s SATISFIABLE\n";
        printSolution(m_maxCount, 't');
        exit(0);
      } else {
        m_out << "i " << ++m_countUpdateMaxCount << " " << std::fixed
              << std::setprecision(2) << getTimer() << " ";
        m_out << std::fixed << std::setprecision(50) << m_maxCount.count
              << "\n";
      }
    }
  }  // updateBound

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
                          MaxSharpSatResult &result) {
    if (m_stopProcess) return;

    showRun(out);
    m_nbCallCall++;

    // is the problem still satisfiable?
    if (!m_solver->solve(setOfVar)) {
      result = {m_aggregator.sumIdentity(), NULL};
      return;
    }

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;
    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);

    // init the returned result.
    result.valuation = getArray();
    setZeroArray(result.valuation);

    // set a valuation for fixed variables.
    T saveCount = m_scale.count;
    T fixCount = m_aggregator.mulIdentity(),
      fixInd = m_aggregator.mulIdentity();

    // std::cout << "==> " << m_solver->getAssumption().size() << '\n';

    for (auto &v : freeVariable)
      if (m_isMaxDecisionVariable[v]) {
        Lit l = Lit::makeLitTrue(v);
        if (m_aggregator.isGreaterThan(~l, l)) l = ~l;

        setBit(m_scale.valuation, m_redirectionPos[v], 1 - l.sign());
        setBit(result.valuation, m_redirectionPos[v], 1 - l.sign());
        fixCount = fixCount * m_aggregator.getWeightLit(l);
      } else if (m_isDecisionVariable[v]) {
        fixInd = fixInd * m_aggregator.getWeightVar(v);
        // std::cout << "free " << v << '\n';
      }

    // consider the unit literals that belong to max
    for (auto &l : unitsLit)
      if (m_isMaxDecisionVariable[l.var()]) {
        fixCount = fixCount * m_aggregator.getWeightLit(l);
        setBit(m_scale.valuation, m_redirectionPos[l.var()], 1 - l.sign());
        setBit(result.valuation, m_redirectionPos[l.var()], 1 - l.sign());
      } else if (m_isDecisionVariable[l.var()]) {
        fixInd = fixInd * m_aggregator.getWeightLit(l);
        // std::cout << "fix " << l << ' ' << m_aggregator.getWeightLit(l) <<
        // '\n';
      }

    result.count = fixCount;
    m_scale.count = m_scale.count * fixCount * fixInd;

    expelNoDecisionVar(freeVariable);
    bool wasUnderAnd = m_isUnderAnd;

    // dig for an assignment for each component (execpt the first one).
    m_isUnderAnd = wasUnderAnd || nbComponent > 1;

    // consider each connected component.
    T mustMultiply = m_aggregator.mulIdentity();
    if (nbComponent) {
      m_nbSplitMax += (nbComponent > 1) ? nbComponent : 0;
      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];

        TmpEntry<MaxSharpSatResult> cb =
            m_cacheMaxActivated ? m_cacheMax->searchInCache(connected)
                                : NULL_CACHE_ENTRY;

        if (cb.defined) {
          mustMultiply = cb.getValue().count;
          if (cb.getValue().valuation)
            orOnMaxVar(connected, result.valuation, cb.getValue().valuation);
        } else {
          MaxSharpSatResult tmpResult;
          searchMaxSharpSatDecision(connected, out, tmpResult);
          if (m_cacheMaxActivated) m_cacheMax->addInCache(cb, tmpResult);
          mustMultiply = tmpResult.count;
          if (tmpResult.valuation)
            orOnMaxVar(connected, result.valuation, tmpResult.valuation);
        }

        result.count = result.count * mustMultiply;
      }
    }  // else we have a tautology

    m_isUnderAnd = wasUnderAnd;
    m_scale.count = saveCount;

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit);

    // update the global maxcount if needed.
    m_scale.count = m_scale.count * fixInd;
    updateBound(result, setOfVar);
    m_scale.count = saveCount;
  }  // searchMaxValuation

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
    if (m_solutionFound && m_heuristicMax == "best") {
      return 1 - getBit(m_maxCount.valuation, m_redirectionPos[v]);
    }

    return m_hPhaseMax->selectPhase(v);
  }  // selectPhase

  /**
   * This function select a variable and compile a decision node.
   *
   * @param[in] connected, the set of variable present in the current
   * problem.
   * @param[in] out, the stream we use to print out logs.
   * @param[out] result, the best solution found.
   */
  void searchMaxSharpSatDecision(std::vector<Var> &connected, std::ostream &out,
                                 MaxSharpSatResult &result) {
    if (m_stopProcess) return;

    // search the next variable to branch on
    Var v =
        m_hVarMax->selectVariable(connected, *m_specs, m_isMaxDecisionVariable);

    if (v == var_Undef) {
      std::vector<Lit> unitsLit;
      std::vector<Var> freeVar;

#if 0
      static int cpt = 0;
      std::cout << "m ";
      for (auto &l : m_solver->getAssumption()) std::cout << l << ' ';
      std::cout << "0\n";
      cpt++;
      if (cpt > 100000) exit(0);
#endif

      float startInd = getTimer();
      result.count = countInd_(connected, unitsLit, freeVar, out);
      m_aggregator.multiplyUnitFree(result.count, unitsLit, freeVar);

      m_accTime += getTimer() - startInd;
      result.valuation = NULL;
      return;
    }

    Lit l = Lit::makeLit(v, selectPhase(v));
    m_nbDecisionNode++;

    // consider the two value for l
    DataBranch<T> b[2];
    MaxSharpSatResult res[2];

    float startTime = getTimer();

    // search max#sat for the first phase.
    assert(!m_solver->isInAssumption(l.var()));
    m_solver->pushAssumption(l);
    searchMaxValuation(connected, b[0].unitLits, b[0].freeVars, out, res[0]);

    m_solver->popAssumption();
    b[0].d = res[0].count;
    m_aggregator.multiplyUnitFree(b[0].d, b[0].unitLits, b[0].freeVars);

    // search max#sat for the next phase.
    if (m_solver->isInAssumption(l))
      res[1].count = m_aggregator.sumIdentity();
    else if (m_solver->isInAssumption(~l))
      searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out, res[1]);
    else {
      m_solver->pushAssumption(~l);
      searchMaxValuation(connected, b[1].unitLits, b[1].freeVars, out, res[1]);
      m_solver->popAssumption();
    }

    b[1].d = res[1].count;
    m_aggregator.multiplyUnitFree(b[1].d, b[1].unitLits, b[1].freeVars);

    // aggregation with max.
    result.count = (b[0].d > b[1].d) ? b[0].d : b[1].d;
    result.valuation = (b[0].d > b[1].d) ? res[0].valuation : res[1].valuation;
  }  // searchMaxSharpSatDecision

  /**
   * @brief Count the number of projected models.
   *
   * @param setOfVar is the current set of considered variables
   * @param unitsLit is the set of unit literal detected at this level
   * @param freeVariable is the variables which become free decision node
   * @param out is the stream we use to print out logs.
   *
   * \return the number of models.
   */
  T countInd_(std::vector<Var> &setOfVar, std::vector<Lit> &unitsLit,
              std::vector<Var> &freeVariable, std::ostream &out) {
    if (m_stopProcess) return m_aggregator.sumIdentity();

    showRun(out);
    m_nbCallProj++;

    if (!m_solver->solve(setOfVar)) return m_aggregator.sumIdentity();

    m_solver->whichAreUnits(setOfVar, unitsLit);  // collect unit literals
    m_specs->preUpdate(unitsLit);

    // compute the connected composant
    std::vector<std::vector<Var>> varConnected;

    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);
    expelNoDecisionVar(freeVariable);

    // consider each connected component.
    T result = m_aggregator.mulIdentity();
    if (nbComponent) {
      m_nbSplitInd += (nbComponent > 1) ? nbComponent : 0;

      for (int cp = 0; cp < nbComponent; cp++) {
        std::vector<Var> &connected = varConnected[cp];

        TmpEntry<T> cb = m_cacheIndActivated
                             ? m_cacheInd->searchInCache(connected)
                             : NULL_CACHE_ENTRY_T;

        if (cb.defined)
          result = result * cb.getValue();
        else {
          T curr = countIndDecisionNode(connected, out);
          if (m_cacheIndActivated) m_cacheInd->addInCache(cb, curr);
          result = result * curr;
        }
      }
    }  // else we have a tautology

    m_specs->postUpdate(unitsLit);
    expelNoDecisionLit(unitsLit);

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
  T countIndDecisionNode(std::vector<Var> &connected, std::ostream &out) {
    if (m_stopProcess) return m_aggregator.sumIdentity();

    // search the next variable to branch on
    ListLit lits;
    m_heuristicInd->selectLitSet(connected, lits);
    if (!lits.size()) return m_aggregator.mulIdentity();
    m_nbDecisionNode++;

    T ret = m_aggregator.sumIdentity();
    DataBranch<T> b;

    unsigned sizeAssum = m_solver->sizeAssumption();
    for (unsigned i = 0; i <= lits.size(); i++) {
      b.unitLits.resize(0);
      b.freeVars.resize(0);

      if (i != 0) {
        m_solver->popAssumption();
        assert(!m_specs->varIsAssigned(lits[i - 1].var()));
        m_solver->pushAssumption(~lits[i - 1]);
        if (lits.size() > 1 && !m_solver->solve(connected)) break;
      }

      if (i != lits.size()) {
        assert(!m_specs->varIsAssigned(lits[i].var()));
        m_solver->pushAssumption(lits[i]);
      }

      b.d = countInd_(connected, b.unitLits, b.freeVars, out);
      m_aggregator.multiplyUnitFree(b.d, b.unitLits, b.freeVars);
      ret = ret + b.d;
    }

    // reinit some variables.
    assert(m_solver->sizeAssumption() > sizeAssum);
    m_solver->popAssumption(m_solver->sizeAssumption() - sizeAssum);

    return ret;
  }  // computeIndDecisionNode

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
        (warmStart && !m_solver->warmStart(29, 11, setOfVar, m_out))) {
      result = {m_aggregator.sumIdentity(), NULL};
      return;
    }

    DataBranch<T> b;
    searchMaxValuation(setOfVar, b.unitLits, b.freeVars, out, result);
    assert(result.valuation);

    if (!m_stopProcess)
      m_aggregator.multiplyUnitFree(result.count, b.unitLits, b.freeVars);
  }  // compute

 public:
  /**
   * @brief Stop the current search and print out the best solution found so
   * far.
   *
   */
  void interrupt() override {
    if (m_maxCount.count < m_aggregator.min())
      std::cout << "c No solution found so far\n";
    else {
      std::cout << "c Processus interrupted, here is the best solution found\n";
      printSolution(m_maxCount, 'k');
    }
  }  // interrupt

  /**
   * @brief Search for the instantiation of the variables of
   * m_problem->getMaxVar() that maximize the number of the remaining
   * variables where the variables not belonging to m_problem->getIndVar()
   * are existantially quantified.
   *
   */
  void run() {
    std::vector<Var> setOfVar;
    for (int i = 1; i <= m_specs->getNbVariable(); i++) setOfVar.push_back(i);

    MaxSharpSatResult result;
    compute(setOfVar, m_out, result);
    printFinalStats(m_out);
    if (!m_stopProcess) {
      printSolution(result, 'o');
    }
  }  // run

};  // end class
}  // namespace d4
