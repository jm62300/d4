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
#include <unordered_map>

#include "DpllStyleMethod.hpp"
#include "MethodManager.hpp"
#include "SemiringConcept.hpp"
#include "src/caching/CacheManager.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/methods/Counter.hpp"
#include "src/options/methods/OptionProjMcMethod.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {

template <NumberType T, SemiringPolicy<T> O>
class ProjMCMethod : public MethodManager {
 private:
  struct CoupleNotProjClauseSelector {
    std::vector<Lit> clause;
    Lit selector;

    void display() {
      std::cout << selector << " : ";
      for (const auto& l : clause) std::cout << l << " ";
      std::cout << "\n";
    }
  };

  // constants
  const unsigned c_NB_SEP = 92;
  const long unsigned c_MASK_HEADER = 1048575;
  const long unsigned c_MASK_SHOWRUN = ((2 << 13) - 1);
  const unsigned c_WIDTH_PRINT_COLUMN = 12;

  std::ostream m_out;
  std::ostream m_outCounter;

  std::vector<Lit> m_selectors;
  std::vector<std::vector<Lit>> m_selectorToProjClause;
  std::vector<std::vector<unsigned>> m_selectorToNProjClause;
  std::vector<bool> m_isProjectedVar;
  std::vector<bool> m_isSelector;
  std::vector<int> m_marked;
  std::vector<bool> m_flag;
  std::unordered_map<std::string, Lit> mapProjClSelector;
  std::vector<CoupleNotProjClauseSelector> notProjClauses;

  FormulaManager* m_specs;
  WrapperSolver* m_solver;
  Counter<T>* m_counter;
  CacheManager<T>* m_cache;

  O m_semiringOps;

  long unsigned m_nbCallRec;
  long unsigned m_nbSplit;

  bool m_refinement;

 public:
  /**
     Constructor.

     @param[in] options, the method options.
     @param[in] problem, the input problem.
     @param[in] out, the output stream for logs.
   */
  ProjMCMethod(const OptionProjMcMethod& options, const ProblemManager& problem,
               std::ostream& out)
      : m_out(nullptr), m_outCounter(nullptr) {
    m_nbCallRec = m_nbSplit = 0;

    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());
    m_out.basic_ios<char>::rdbuf(out.rdbuf());

    // mark the projected variables.
    const std::vector<Var>& projVars = problem.getQuantification().empty()
                                           ? std::vector<Var>{}
                                           : problem.getQuantification()[0];
    m_isProjectedVar.resize(problem.getNbVar() + 1, false);
    m_isSelector.resize(problem.getNbVar() + 1, false);
    for (auto v : projVars) m_isProjectedVar[v] = true;

    m_refinement = options.refinement.get();
    m_out << "c [PROJ MC]" << options << "\n";

    std::vector<std::vector<Lit>> projClause, nprojClause, mix;
    partitionFormula(problem, m_isProjectedVar, projClause, nprojClause, mix);

    // split the mixed clauses.
    unsigned idxVar = problem.getNbVar() + 1;
    manageMixedClauses(mix, m_isProjectedVar, projClause, nprojClause,
                       m_selectors, idxVar);

    // prepare the SAT solver with all clauses (proj + nproj).
    std::vector<std::vector<Lit>> satSolverClauses = projClause;
    for (auto& cl : nprojClause) satSolverClauses.push_back(cl);
    initSatSolver(options.optionSolver, problem, satSolverClauses, idxVar - 1);

    // prepare the spec manager with the projected formula (including
    // selectors).
    initSpecManager(options.optionSpecs, problem, projClause, idxVar - 1);

    // prepare the cache.
    m_cache = CacheManager<T>::makeCacheManager(options.optionCache, idxVar - 1,
                                                m_specs, m_out);

    // init the clock time.
    initTimer();

    // prepare the counter for the projected formula.
    initCounter(options.optionCounter, problem, projClause, idxVar - 1);

    m_marked.resize(idxVar + 1, -1);
    m_flag.resize((idxVar + 1) << 1, false);

    // init semiring with original problem weights.
    m_semiringOps = O(problem.getNbVar(), problem.getWeightMap());
  }  // constructor

  /**
     Destructor.
   */
  ~ProjMCMethod() {
    delete m_counter;
    delete m_solver;
    delete m_specs;
    delete m_cache;
  }  // destructor

 private:
  /**
     Build a ProblemManager from a clause list.

     @param[in] nbVar, the number of variables.
     @param[in] quantifications, the variable quantification blocks.
     @param[in] weightMap, the weight map.
     @param[in] clauses, the clause list.
     @return a ProblemManager owning those clauses.
   */
  static ProblemManager buildCnfProblem(
      unsigned nbVar, const std::vector<std::vector<Var>>& quantifications,
      const std::map<Lit, std::string>& weightMap,
      const std::vector<std::vector<Lit>>& clauses) {
    std::vector<BcGate> gates;
    gates.reserve(clauses.size());
    for (const auto& cl : clauses)
      gates.push_back({cl, lit_Undef, BcGateType::CLAUSE});
    std::ostream nullStream(nullptr);
    return ProblemManager("cnf", nbVar, quantifications, weightMap, gates,
                          nullStream);
  }

  /**
     Init the inner counter with the projected clauses.

     @param[in] options, the option to create the counter.
     @param[in] problem, the original input problem.
     @param[in] clauses, the projected clause set (including selector clauses).
     @param[in] nbVar, the number of variables (including selectors).
   */
  void initCounter(const OptionDpllStyleMethod& options,
                   const ProblemManager& problem,
                   std::vector<std::vector<Lit>>& clauses, unsigned nbVar) {
#if DEBUG
    m_outCounter.copyfmt(m_out);
    m_outCounter.clear(m_out.rdstate());
    m_outCounter.basic_ios<char>::rdbuf(m_out.rdbuf());
#else
    m_outCounter.setstate(std::ios_base::badbit);
#endif
    // Empty projection: counter counts all models (selectors are fixed by
    // assumptions so weight=1 per selector).
    std::vector<std::vector<Var>> emptyQuant = {std::vector<Var>{}};
    ProblemManager counterProblem =
        buildCnfProblem(nbVar, emptyQuant, problem.getWeightMap(), clauses);

    m_out << "c [PROJ MC] Create an external counter\n";
    m_counter =
        new DpllStyleMethod<T, O>(options, counterProblem, m_outCounter);
  }  // initCounter

  /**
     Init the SAT solver with the full clause set (proj + nproj).

     @param[in] options, the option to create the SAT solver.
     @param[in] problem, the original input problem (used for type detection).
     @param[in] clauses, the combined clause set.
     @param[in] nbVar, the number of variables (including selectors).
   */
  void initSatSolver(const OptionSolver& options, const ProblemManager& problem,
                     std::vector<std::vector<Lit>>& clauses, unsigned nbVar) {
    m_solver = WrapperSolver::makeWrapperSolver(options, problem, m_out);
    assert(m_solver);

    ProblemManager satProblem = buildCnfProblem(
        nbVar, problem.getQuantification(), problem.getWeightMap(), clauses);
    m_solver->initSolver(satProblem);
    m_solver->setNeedModel(true);
  }  // initSatSolver

  /**
     Init the spec manager with the projected formula (including selectors).

     @param[in] options, spec manager options.
     @param[in] problem, the original input problem.
     @param[in] projClauses, the projected clauses (with selector vars).
     @param[in] nbVar, the number of variables (including selectors).
   */
  void initSpecManager(const OptionSpecManager& options,
                       const ProblemManager& problem,
                       const std::vector<std::vector<Lit>>& projClauses,
                       unsigned nbVar) {
    std::vector<std::vector<Var>> emptyQuant = {std::vector<Var>{}};
    ProblemManager projProblem =
        buildCnfProblem(nbVar, emptyQuant, problem.getWeightMap(), projClauses);
    m_specs = FormulaManager::makeFormulaManager(options, projProblem, m_out);
  }  // initSpecManager

  /**
     Partition the formula into three sets: projected-only clauses,
     non-projected-only clauses, and mixed clauses.

     @param[in] problem, the problem to partition.
     @param[in] isProjectedVar, boolean vector marking projected variables.
     @param[out] projClause, clauses containing only projected variables.
     @param[out] nprojClause, clauses containing no projected variables.
     @param[out] mix, clauses containing both.
   */
  void partitionFormula(const ProblemManager& problem,
                        std::vector<bool>& isProjectedVar,
                        std::vector<std::vector<Lit>>& projClause,
                        std::vector<std::vector<Lit>>& nprojClause,
                        std::vector<std::vector<Lit>>& mix) {
    for (const auto& gate : problem.getGates()) {
      assert(gate.gateType == BcGateType::CLAUSE);
      const std::vector<Lit>& cl = gate.input;

      unsigned nbp = 0, nbn = 0;
      for (auto l : cl) {
        if (isProjectedVar[l.var()])
          nbp++;
        else
          nbn++;
        if (nbp && nbn) break;
      }

      if (nbp && !nbn)
        projClause.push_back(cl);
      else if (!nbp && nbn)
        nprojClause.push_back(cl);
      else
        mix.push_back(cl);
    }
  }  // partitionFormula

  /**
     Manage the mixed clauses by adding a selector variable to separate the
     projected and non-projected parts.

     @param[in] mix, clauses containing both projected and non-projected vars.
     @param[in] isProjectedVar, boolean vector marking projected variables.
     @param[out] projClause, receives extended projected clauses (clp ∨ s).
     @param[out] nprojClause, receives extended non-proj clauses (clnp ∨ ¬s).
     @param[out] selectors, the added selector literals.
     @param[in,out] nextVar, index of the next available variable.
   */
  void manageMixedClauses(std::vector<std::vector<Lit>>& mix,
                          std::vector<bool>& isProjectedVar,
                          std::vector<std::vector<Lit>>& projClause,
                          std::vector<std::vector<Lit>>& nprojClause,
                          std::vector<Lit>& selectors, unsigned& nextVar) {
    for (auto& cl : mix) {
      std::vector<Lit> clp, clnp;
      std::string key = "";
      for (auto& l : cl)
        if (!isProjectedVar[l.var()])
          clnp.push_back(l);
        else {
          key += "." + std::to_string(l.intern());
          clp.push_back(l);
        }

      Lit s = lit_Undef;
      if (mapProjClSelector.count(key) > 0)
        s = mapProjClSelector[key];
      else {
        s = Lit::makeLitTrue(nextVar++);
        if (s.var() >= (int)m_isSelector.size())
          m_isSelector.resize(s.var() + 1, false);
        m_isSelector[s.var()] = true;

        mapProjClSelector[key] = s;

        if ((int)m_selectorToProjClause.size() <= s.var()) {
          m_selectorToProjClause.resize(s.var() + 1, std::vector<Lit>());
          m_selectorToNProjClause.resize(s.var() + 1, std::vector<unsigned>());
        }
        m_selectorToProjClause[s.var()] = clp;

        clp.push_back(s);
        projClause.push_back(clp);

        m_isProjectedVar.resize(nextVar);
        m_isProjectedVar[s.var()] = false;
      }

      // link the selector to the non-projected part of the clause.
      m_selectorToNProjClause[s.var()].push_back(notProjClauses.size());
      notProjClauses.push_back({clnp, s});
      clnp.push_back(~s);
      nprojClause.push_back(clnp);
    }
  }  // manageMixedClauses

  /**
     Extract the selector literals corresponding to non-projected clauses
     that are falsified under the given model.

     @param[in] setOfVar, the set of variables to inspect.
     @param[in] model, the current interpretation.
     @param[out] selector, the returned selectors with their required values.
   */
  void extractSelectorFalsifiedNProj(std::vector<Var>& setOfVar,
                                     std::vector<lbool>& model,
                                     std::vector<Lit>& selector) {
    for (auto& v : setOfVar) {
      if (v >= (int)m_isSelector.size() || !m_isSelector[v]) continue;
      if (m_solver->isInAssumption(v)) continue;

      bool allSAT = true;
      for (auto idx : m_selectorToNProjClause[v]) {
        const std::vector<Lit>& cl = notProjClauses[idx].clause;
        bool isSAT = false;
        for (auto& l : cl) {
          if (model[l.var()] == l_Undef)
            continue;
          else if (l.sign())
            isSAT = model[l.var()] == l_False;
          else
            isSAT = model[l.var()] == l_True;

          if (isSAT) break;
        }

        if (!(allSAT = isSAT)) break;
      }

      selector.push_back(Lit::makeLit(v, !allSAT));
    }
  }  // extractSelectorFalsifiedNProj

  /**
     Remove from litList and varList the elements that are not projected
     variables.

     @param[out] litList, the literal list to filter in-place.
     @param[out] varList, the variable list to filter in-place.
   */
  void expelNoProjectedElement(std::vector<Lit>& litList,
                               std::vector<Var>& varList) {
    unsigned j = 0;
    for (unsigned i = 0; i < litList.size(); i++)
      if (m_isProjectedVar[litList[i].var()]) litList[j++] = litList[i];
    litList.resize(j);

    j = 0;
    for (unsigned i = 0; i < varList.size(); i++)
      if (m_isProjectedVar[varList[i]]) varList[j++] = varList[i];
    varList.resize(j);
  }  // expelNoProjectedElement

  /**
     Compute the weight contribution of projected unit literals and free
     projected variables using the semiring.

     @param[in] unitLits, projected unit literals.
     @param[in] freeVars, projected free variables.
     @return the multiplicative weight factor.
   */
  T computeWeightUnitFree(const std::vector<Lit>& unitLits,
                          const std::vector<Var>& freeVars) {
    T w = m_semiringOps.zero();
    m_semiringOps.add(w, m_semiringOps.one(), unitLits, freeVars);
    return w;
  }  // computeWeightUnitFree

  /**
     Try to reduce the number of falsified selectors using SAT calls.

     @param[in] setOfVar, the set of variables to consider.
     @param[out] selector, the selector list to reduce.
   */
  void refine(std::vector<Var>& setOfVar, std::vector<Lit>& selector) {
    unsigned initSizeAssumption = m_solver->sizeAssumption();

    // split between true (satisfied) and false (unsatisfied) selectors.
    std::vector<Lit> falseSelector;
    unsigned j = 0;
    for (unsigned i = 0; i < selector.size(); i++) {
      Lit l = selector[i];
      if (l.sign())
        falseSelector.push_back(l);
      else {
        selector[j++] = selector[i];
        assert(!m_solver->isInAssumption(l.var()));
        m_solver->pushAssumption(l);
      }
    }
    selector.resize(j);

    // reintegrate false selectors, flipping if possible.
    for (unsigned i = 0; i < falseSelector.size(); i++) {
      Lit l = falseSelector[i];
      if (m_solver->isInAssumption(l.var())) {
        selector.push_back(m_solver->isInAssumption(l) ? l : ~l);
        continue;
      }

      m_solver->pushAssumption(~l);
      std::vector<Lit> units;
      if (m_solver->solve(setOfVar, units)) {
        std::vector<lbool>& model = m_solver->getModel();
        selector.push_back(~l);

        unsigned kj = i + 1;
        for (unsigned ki = i + 1; ki < falseSelector.size(); ki++) {
          bool allSAT = true;
          for (auto idx : m_selectorToNProjClause[falseSelector[ki].var()]) {
            const std::vector<Lit>& cl = notProjClauses[idx].clause;
            bool isSAT = false;
            for (auto& kl : cl) {
              if (model[kl.var()] == l_Undef)
                continue;
              else if (kl.sign())
                isSAT = model[kl.var()] == l_False;
              else
                isSAT = model[kl.var()] == l_True;

              if (isSAT) break;
            }

            if (!(allSAT = isSAT)) break;
          }

          if (allSAT) {
            selector.push_back(~falseSelector[ki]);
            assert(!m_solver->isInAssumption(falseSelector[ki]));
            if (!m_solver->isInAssumption(~falseSelector[ki]))
              m_solver->pushAssumption(~falseSelector[ki]);
          } else
            falseSelector[kj++] = falseSelector[ki];
        }
        falseSelector.resize(kj);
      } else {
        selector.push_back(l);
        m_solver->popAssumption(1);
        m_solver->pushAssumption(l);
      }
    }

    m_solver->popAssumption(m_solver->sizeAssumption() - initSizeAssumption);
  }  // refine

  /**
     Compute the projected model count over the given set of variables.

     @param[in] setOfVar, the set of variables to consider.
     @param[in] out, the stream for logs.
     @return the projected model count.
   */
  T compute_(std::vector<Var>& setOfVar, std::ostream& out) {
    std::cout << "compute\n";

    showRun(out);
    m_nbCallRec++;

    unsigned initSizeAssumption = m_solver->sizeAssumption();
    std::vector<Lit> unitLits;
    if (!m_solver->solve(setOfVar, unitLits)) return T(0);

    m_specs->preUpdate(unitLits);

    // add the unit literals to the assumption.
    for (auto& l : unitLits) {
      assert(!m_solver->isInAssumption(~l));
      if (!m_solver->isInAssumption(l)) m_solver->pushAssumption(l);
    }

    std::vector<Var> reallyPresent, freeVariable;
    std::vector<std::vector<Var>> varConnected;
    int nbComponent = m_specs->computeConnectedComponent(varConnected, setOfVar,
                                                         freeVariable);

    // collect vars that are in a connected component.
    reallyPresent.reserve(setOfVar.size() - freeVariable.size());
    for (auto& comp : varConnected)
      for (auto& v : comp) reallyPresent.push_back(v);

    // extract the projected variables in the component.
    std::vector<Var> projectSetOfVar;
    for (auto& v : reallyPresent)
      if (m_isProjectedVar[v]) projectSetOfVar.push_back(v);

    T ret = T(1);
    if (projectSetOfVar.size()) {
      if (nbComponent > 1) {
        std::cout << "more than one component\n";
        for (auto& component : varConnected) ret *= compute_(component, out);
        m_nbSplit += nbComponent;
      } else if (nbComponent == 1) {
        // extract selector values for non-projected constraints.
        std::vector<Lit> selector;
        extractSelectorFalsifiedNProj(reallyPresent, m_solver->getModel(),
                                      selector);
        if (m_refinement) refine(reallyPresent, selector);

        TmpEntry<T> cb = m_cache->searchInCache(reallyPresent);
        if (cb.defined)
          ret = cb.getValue();
        else {
          // count projected models under current selector assignments.
          std::vector<Lit> nextAssums(m_solver->getAssumption());
          std::cout << "==========> ";
          for (auto& l : nextAssums) std::cout << l.human() << ' ';
          std::cout << '\n';

          for (auto& s : selector) {
            assert(!m_solver->isInAssumption(s.var()));
            if (!m_solver->isInAssumption(s)) nextAssums.push_back(s);
          }
          ret = m_counter->count(reallyPresent, nextAssums, m_outCounter);

          // for each unsatisfied selector, add models where the projected
          // clause forces satisfaction (inclusion-exclusion step).
          unsigned j = 0;
          for (unsigned i = 0; i < selector.size(); i++)
            if (selector[i].sign()) selector[j++] = selector[i];
          selector.resize(j);

          for (auto& s : selector) {
            unsigned countPushInAssumption = 0;
            auto& cl = m_selectorToProjClause[s.var()];
            bool isUnsat = false;
            for (auto& l : cl) {
              if (m_solver->isInAssumption(l)) {
                isUnsat = true;
                break;
              }
              if (!m_solver->isInAssumption(~l)) {
                m_solver->pushAssumption(~l);
                countPushInAssumption++;
              }
            }

            if (!isUnsat) ret += compute_(reallyPresent, out);
            m_solver->popAssumption(countPushInAssumption);
            if (m_solver->isInAssumption(~s)) break;  // UNSAT.
            if (!m_solver->isInAssumption(s)) m_solver->pushAssumption(s);
          }

          m_solver->popAssumption(selector.size());
          m_cache->addInCache(cb, ret);
        }
      }
    }

    m_specs->postUpdate(unitLits);
    expelNoProjectedElement(unitLits, freeVariable);

    m_solver->popAssumption(m_solver->sizeAssumption() - initSizeAssumption);
    return ret * computeWeightUnitFree(unitLits, freeVariable);
  }  // compute_

  /**
     Entry point for the recursive computation.

     @param[in] out, the output stream.
     @return the projected model count.
   */
  T compute(std::ostream& out) {
    std::vector<Var> setOfVar;
    for (unsigned i = 1; i <= (unsigned)m_specs->getNbVariable(); i++)
      setOfVar.push_back(i);
    return compute_(setOfVar, out);
  }  // compute

  /**
   Print out the running statistics.

   @param[in] out, the stream to write to.
*/
  inline void showInter(std::ostream& out) {
    out << "c [PROJMC] " << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << m_nbCallRec << std::fixed << std::setprecision(2) << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << getTimer() << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << m_cache->getNbPositiveHit() << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << m_cache->getNbNegativeHit() << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << m_cache->usedMemory() << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << m_nbSplit << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << MemoryStat::memUsedPeak()
        << "|\n";
  }  // showInter

  inline void separator(std::ostream& out) {
    out << "c [PROJMC] ";
    for (unsigned i = 0; i < c_NB_SEP; i++) out << "-";
    out << "\n";
  }  // separator

  inline void showHeader(std::ostream& out) {
    separator(out);
    out << "c [PROJMC] " << "|" << std::setw(c_WIDTH_PRINT_COLUMN)
        << "#rec. call" << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "time"
        << "|" << std::setw(c_WIDTH_PRINT_COLUMN) << "#posHit" << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << "#negHit" << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << "memory" << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << "#split" << "|"
        << std::setw(c_WIDTH_PRINT_COLUMN) << "mem(MB)" << "|\n";
    separator(out);
  }  // showHeader

  inline void showRun(std::ostream& out) {
    if (!(m_nbCallRec & (c_MASK_HEADER))) showHeader(out);
    if (m_nbCallRec && !(m_nbCallRec & c_MASK_SHOWRUN)) showInter(out);
  }  // showRun

  inline void printFinalStats(std::ostream& out) {
    out << "c [PROJMC] \n";
    out << "c [PROJMC] \033[1m\033[31mStatistics \033[0m\n";
    out << "c [PROJMC] \033[33mCompilation Information\033[0m\n";
    out << "c [PROJMC]\n";
    m_cache->printCacheInformation(out);
    out << "c [PROJMC] Final time: " << getTimer() << "\n";
    out << "c\n";
  }  // printFinalStats

 public:
  /**
     Run the ProjMC algorithm.

     @return the projected model count.
   */
  T run() {
    T res = compute(m_out);
    printFinalStats(m_out);
    return res;
  }  // run
};
}  // namespace d4
