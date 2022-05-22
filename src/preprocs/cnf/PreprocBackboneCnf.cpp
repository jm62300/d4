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

#include <csignal>
#include <ctime>

#include "3rdParty/bipe/srcBipe/methods/Backbone.hpp"
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
                                        LastBreathPreproc &lastBreath,
                                        unsigned timeout) {
  // init the solver.
  ws->initSolver(*pin);
  ws->setNeedModel(true);

  if (!ws->solve()) return pin->getUnsatProblem();
  lastBreath.panic = ws->getNbConflict() > 100000;

  // get the activity given by the solver.
  lastBreath.countConflict.resize(pin->getNbVar() + 1, 0);
  for (unsigned i = 1; i <= pin->getNbVar(); i++)
    lastBreath.countConflict[i] = ws->getCountConflict(i);

  // get the unit.
  std::vector<Lit> units;
  ws->getUnits(units);

  // create the problem regarding the bipe library.
  std::vector<Var> protect, selected;
  if (pin->getSelectedVar().size())
    selected = pin->getSelectedVar();
  else
    for (unsigned i = 1; i <= pin->getNbVar(); i++)
      if (pin->getWeightLit(Lit::makeLitTrue(i)) ==
          pin->getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);

  bipe::Problem pb(pin->getNbVar(), pin->getWeightLit(), selected, protect);

  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  std::vector<std::vector<bipe::Lit>> &clauses = pb.getClauses();
  for (auto l : units)
    clauses.push_back({bipe::Lit::makeLit(l.var(), l.sign())});
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  // call the preprocessor to compute the backbone.
  bipe::Backbone bb;
  std::vector<bipe::Gate> gates;
  std::vector<std::vector<bipe::lbool>> setOfModels;

  std::cerr << "c [PREPOC BACKBONE] Is running ...\n";
  PreprocManager::s_isRunning = &bb;

  // change the handler.
  void (*handler)(int) = [](int s) {
    if (PreprocManager::s_isRunning)
      ((bipe::Backbone *)PreprocManager::s_isRunning)->interrupt();
  };
  signal(SIGALRM, handler);
  alarm(timeout);

  bool res = bb.run(pb, gates, 0, std::cout, "Glucose_bipe", true, setOfModels);

  if (!res) {
    std::cerr << "c [PREPOC BACKBONE] We already checked that is SAT Oo\n";
    exit(-1);
  }

  // the list of unit literals.
  units.clear();
  for (auto g : gates)
    units.push_back(Lit::makeLit(g.output.var(), g.output.sign()));

  std::cout << "c [PREPOC BACKBONE] Backone size: " << units.size() << "\n";
  std::cout << "c [PREPOC BACKBONE] Panic in the preprocessing: "
            << lastBreath.panic << "\n";

  m_isRunning = NULL;
  alarm(0);
  return pin->getConditionedFormula(units);
}  // run
}  // namespace d4
