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

#include "PreprocEquiv.hpp"

#include <csignal>

#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocEquiv::PreprocEquiv(po::variables_map &vm, std::string &method,
                           int nbIteration, std::ostream &out) {
  ws = WrapperSolver::makeWrapperSolverPreproc(vm, out);
  m_method = method;
  m_nbIteration = nbIteration;
}  // constructor

/**
   Destructor.
 */
PreprocEquiv::~PreprocEquiv() { delete ws; }  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocEquiv::run(ProblemManager *pin,
                                  LastBreathPreproc &lastBreath,
                                  unsigned timeout) {
  std::cout << "c [EQUIV PREPROC] Start\n";
  ws->initSolver(*pin);
  lastBreath.panic = 0;
  lastBreath.countConflict.resize(pin->getNbVar() + 1, 0);

  if (!ws->solve()) return pin->getUnsatProblem();
  lastBreath.panic = ws->getNbConflict() > 100000;

  // get the activity given by the solver.
  for (unsigned i = 1; i <= pin->getNbVar(); i++)
    lastBreath.countConflict[i] = ws->getCountConflict(i);

  std::vector<Lit> units;
  ws->getUnits(units);

  // get the cnf.
  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);

  // compute the backbone.
  std::vector<Var> protect, selected;
  if (pin->getSelectedVar().size())
    selected = pin->getSelectedVar();
  else
    for (unsigned i = 1; i <= pin->getNbVar(); i++)
      if (pin->getWeightLit(Lit::makeLitTrue(i)) ==
          pin->getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);

  bipe::Problem pb(pin->getNbVar(), pin->getWeightLit(), selected, protect);
  std::vector<std::vector<bipe::Lit>> &clauses = pb.getClauses();
  for (auto l : units)
    clauses.push_back({bipe::Lit::makeLit(l.var(), l.sign())});
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  // call the preprocessor to compute the backbone.
  // call the preprocessor to compute the backbone.
  bipe::bipartition::Method bb;
  std::vector<bipe::Gate> gates;
  std::vector<std::vector<bool>> setOfModels;

  std::cerr << "c [PREPOC BACKBONE] Is running for at most " << timeout
            << " seconds\n";
  PreprocManager::s_isRunning = &bb;

  // change the handler.
  void (*handler)(int) = [](int s) {
    if (PreprocManager::s_isRunning)
      ((bipe::bipartition::Method *)PreprocManager::s_isRunning)->interrupt();
  };
  signal(SIGALRM, handler);
  alarm(timeout);

  bool res = bb.simplifyBackbone(
      pb, (bipe::bipartition::OptionBackbone){true, timeout, true, "Glucose"},
      gates, std::cout, setOfModels);

  if (!res) {
    std::cerr << "c [PREPOC BACKBONE] We already checked that is SAT Oo\n";
    exit(-1);
  }

  if (!m_isInterrupted) {
    // the list of unit literals.
    for (auto g : gates)
      clauses.push_back({bipe::Lit::makeLit(g.output.var(), g.output.sign())});

    // create the problem from the reducer side.
    bipe::reducer::Method *rm =
        bipe::reducer::Method::makeMethod("combinaison", std::cout);

    rm->run(pin->getNbVar(), clauses, 10, true, clauses);

    ProblemManagerCnf *ret = new ProblemManagerCnf(
        pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
        pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

    std::vector<std::vector<Lit>> &clausesAfter = ret->getClauses();
    for (auto &cl : clauses) {
      clausesAfter.push_back({});
      for (auto &l : cl)
        clausesAfter.back().push_back(Lit::makeLit(l.var(), l.sign()));
    }

    delete rm;
    return ret;
  } else {
    // the list of unit literals.
    units.clear();
    for (auto g : gates)
      units.push_back(Lit::makeLit(g.output.var(), g.output.sign()));

    m_isRunningBackbone = NULL;
    return pin->getConditionedFormula(units);
  }
}  // run
}  // namespace d4
