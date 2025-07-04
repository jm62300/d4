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

#include "circuit/WrapperCircuitGlucose.hpp"
#include "circuit/WrapperCircuitMinisat.hpp"
#include "cnf/WrapperGlucose.hpp"
#include "cnf/WrapperMinisat.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {
/**
 * @brief WrapperSolver::makeWrapperSolver implementation.
 */
WrapperSolver *WrapperSolver::makeWrapperSolver(const OptionSolver &options,
                                                ProblemManager &p,
                                                std::ostream &out) {
  out << "c [WRAPPER SOLVER]" << options << "\n";
  WrapperSolver *ret = nullptr;

  switch (p.getProblemType()) {
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

  return ret;
}  // makeWrapperSolver

/**
   Prepare the solver by running it a given number of iteration for some queries
   of a given size.

   @param[in] iteration, the number of queries we test.
   @param[in] sizeQuery, the (maximum) size of the queries.
   @param[in] setOfvar, the set of variable we construct the queries on.
   @param[in] out, the place where we print out the information.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperSolver::warmStart(int iteration, int sizeQuery,
                              std::vector<Var> &setOfVar, std::ostream &out) {
  if (!solve()) return false;
  int nbSAT = 0;

  if (setOfVar.size() > 10000) {
    sizeQuery *= 100;
  }

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
    bool res = solve();  // we do not care the result.
    if (res) nbSAT++;
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
