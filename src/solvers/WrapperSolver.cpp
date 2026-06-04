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

#include "WrapperSolver.hpp"

#include <new>

#include "circuit/WrapperCircuitGlucose.hpp"
#include "circuit/WrapperCircuitMinisat.hpp"
#include "cnf/WrapperGlucose.hpp"
#include "cnf/WrapperMinisat.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {

/**
 * @brief Apply solver options to the configurable parameters.
 */
void WrapperSolver::configure(const OptionSolver& opts) {
  m_initBudget = opts.initBudget;
  m_minLimitVar = opts.minLimitVar;
  m_learntFactor = opts.learntFactor;
  m_cadicalRedundantFactor = opts.cadicalRedundantFactor;
}  // configure

/**
 * @brief Initialize CaDiCaL with the given number of variables.
 */
void WrapperSolver::initCadical(unsigned nbVar) {
  m_nbInitVar = nbVar;
  m_cadical.set("inprocessing", 0);
  m_cadical.set("reduceinit", 1000);
  m_cadical.set("reducefactor", 10);
  m_cadical.set("reducetier1glue", 1);
  m_cadical.declare_more_variables(nbVar + 1);
}  // initCadical

/**
 * @brief Rebuild CaDiCaL from scratch using the stored initial clauses.
 */
void WrapperSolver::rebuildCadical() {
  m_cadical.~Solver();
  new (&m_cadical) CaDiCaL::Solver();
  m_cadical.set("inprocessing", 0);
  m_cadical.set("reduceinit", 1000);
  m_cadical.set("reducefactor", 10);
  m_cadical.set("reducetier1glue", 1);
  m_cadical.declare_more_variables(m_nbInitVar + 1);
  for (const auto& cl : m_initClauses) {
    for (auto l : cl) m_cadical.add(l);
    m_cadical.add(0);
  }
}  // rebuildCadical

/**
 * @brief Run the solver, falling back to CaDiCaL when the budget is exceeded.
 */
bool WrapperSolver::solve(std::span<const Var> setOfVar,
                          std::vector<Lit>& units) {
  if (m_activeModel && m_needModel) {
    whichAreUnits(setOfVar, units);
    return true;
  }

  lbool res = runSolver(setOfVar);

  if (res == l_False) {
    m_activeModel = false;
    return false;
  }

  if (res == l_Undef) {
    if (m_cadical.redundant() > (int64_t)(m_cadicalRedundantFactor * m_initClauses.size())) rebuildCadical();

    m_cadical.reset_assumptions();
    for (auto& l : m_assumption) m_cadical.assume(l.human());

    m_activeModel = m_cadical.solve() == 10;
    if (!m_activeModel) return false;

    for (auto v : setOfVar) m_model[v] = m_cadical.val(v) > 0 ? l_True : l_False;
    onCadicalSat(setOfVar);
  } else {
    m_activeModel = true;
  }

  whichAreUnits(setOfVar, units);
  return true;
}  // solve

/**
 * @brief WrapperSolver::makeWrapperSolver implementation.
 */
WrapperSolver* WrapperSolver::makeWrapperSolver(const OptionSolver& options,
                                                const ProblemManager& p,
                                                std::ostream& out) {
  out << "c [WRAPPER SOLVER]" << options << "\n";
  WrapperSolver* ret = nullptr;

  switch (p.getProblemInputType()) {
    case PB_CIRC:
      switch (options.solverName) {
        case MINISAT_CNF: {
          ret = new WrapperCircuitMinisat();
          break;
        }
        case GLUCOSE_CNF: {
          ret = new WrapperCircuitGlucose();
          break;
        }
      }
      break;
    case PB_TCNF:
    case PB_CNF:
    case PB_QBF:
      switch (options.solverName) {
        case MINISAT_CNF: {
          ret = new WrapperMinisat();
          break;
        }
        case GLUCOSE_CNF: {
          ret = new WrapperGlucose();
          break;
        }
      }
      break;
    case PB_NONE:
      std::cerr << "c The problem type is none\n";
      exit(ERROR_BAD_TYPE_PROBLEM);
  }

  if (!ret) std::runtime_error("The SAT solver selected is unknown");
  ret->configure(options);
  return ret;
}  // makeWrapperSolver

/**
 * @brief Prepare the solver by running it for warm-up queries.
 */
bool WrapperSolver::warmStart(int iteration, int sizeQuery,
                              std::vector<Var>& setOfVar, std::ostream& out) {
  std::vector<Lit> units;
  if (!solve(setOfVar, units)) {
    out << "c Proved UNSAT during initialization\n";
    return false;
  }
  int nbSAT = 0;

  if (setOfVar.size() > 10000) sizeQuery *= 100;

  std::vector<Lit> query(sizeQuery);

  for (int nbIte = 0; nbIte < iteration; nbIte++) {
    query.resize(0);
    for (int i = 0; i < sizeQuery; i++) {
      Var v = setOfVar[rand() % setOfVar.size()];
      Lit l = Lit::makeLit(v, rand() & 1);

      bool isIn = false;
      for (unsigned j = 0; !isIn && j < query.size(); j++)
        isIn = l.var() == query[j].var();
      if (!isIn) query.push_back(l);
    }

    setAssumption(query);
    solveLimited(setOfVar, 500);
    restart();
  }

  query.clear();
  setAssumption(query);
  restart();

  out << "c Warm start process (" << sizeQuery << "): " << nbSAT << "/"
      << iteration << "\n";
  return true;
}  // warmStart

}  // namespace d4
