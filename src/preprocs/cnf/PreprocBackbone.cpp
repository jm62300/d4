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
#include "PreprocBackbone.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocBackbone::PreprocBackbone(po::variables_map &vm, std::ostream &out) {
  ws = WrapperSolver::makeWrapperSolverPreproc(vm, out);
} // constructor

/**
   Destructor.
 */
PreprocBackbone::~PreprocBackbone() { delete ws; } // destructor

/**
   The preprocessing itself.

   @param[out] p, the problem we want to preprocess.
 */
ProblemManager *PreprocBackbone::run(ProblemManager &pin) {
  ws->initSolver(pin);
  if (!ws->solve())
    return pin.getUnsatProblem();

  std::vector<Lit> units;
  ws->getUnits(units);
  return pin.getConditionedFormula(units);
} // run
} // namespace d4
