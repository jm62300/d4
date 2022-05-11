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
#include "PreprocBackboneCnf.hpp"

#include <bits/types/clock_t.h>

#include <ctime>

#include "3rdParty/bipe/src/methods/Backbone.hpp"
#include "3rdParty/bipe/src/methods/Method.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocBackboneCnf::PreprocBackboneCnf(po::variables_map &vm,
                                       std::ostream &out) {
  ws = WrapperSolver::makeWrapperSolverPreproc(vm, out);
}  // constructor

/**
   Destructor.
 */
PreprocBackboneCnf::~PreprocBackboneCnf() { delete ws; }  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocBackboneCnf::run(ProblemManager *pin,
                                        LastBreathPreproc &lastBreath) {
  // init the solver.
  ws->initSolver(*pin);
  ws->setNeedModel(true);
  unsigned nbSatCalls = 1;
  unsigned nbFoundUnit = 0;

  if (!ws->solve()) return pin->getUnsatProblem();
  lastBreath.panic = ws->getNbConflict() > 100000;

  // create the problem regarding the bipe library.
  std::vector<Var> protect;
  bipe::Problem pb(pin->getNbVar(), pin->getWeightLit(), pin->getSelectedVar(),
                   protect);

  // call the preprocessor to compute the backbone.
  bipe::Backbone bb;

  std::vector<bipe::Gate> gates;
  std::vector<std::vector<bipe::lbool>> setOfModels;
  bool res = bb.run(pb, gates, -1, std::cout, "glucose", true, setOfModels);

  // the list of unit literals.
  std::vector<Lit> units;
  for (auto g : gates)
    units.push_back(Lit::makeLit(g.output.var(), g.output.sign()));

  std::cout << "c [PREPOC BACKBONE] Backone size: " << units.size() << "\n";
  std::cout << "c [PREPOC BACKBONE] Number of units detected: " << nbFoundUnit
            << "\n";
  std::cout << "c [PREPOC BACKBONE] Panic in the preprocessing: "
            << lastBreath.panic << "\n";

  return pin->getConditionedFormula(units);
}  // run
}  // namespace d4
