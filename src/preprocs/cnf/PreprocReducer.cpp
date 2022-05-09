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

#include "PreprocReducer.hpp"

#include "3rdParty/reducer/src/methods/Method.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
   The constructor.

   @param[in] vm, the options used (solver).
 */
PreprocReducer::PreprocReducer(po::variables_map &vm, std::string &method,
                               int nbIteration, std::ostream &out) {
  ws = WrapperSolver::makeWrapperSolverPreproc(vm, out);
  m_method = method;
  m_nbIteration = nbIteration;
}  // constructor

/**
   Destructor.
 */
PreprocReducer::~PreprocReducer() { delete ws; }  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocReducer::run(ProblemManager *pin,
                                    LastBreathPreproc &lastBreath) {
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

  // prepage the clauses.3
  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  std::vector<std::vector<reducer::Lit>> clauses;
  for (auto l : units)
    clauses.push_back({reducer::Lit::makeLit(l.var(), l.sign())});
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(reducer::Lit::makeLit(l.var(), l.sign()));
  }

  // create the problem from the reducer side.
  reducer::Problem problem(clauses, pcnf.getNbVar(), std::cout, false);
  reducer::Method *reducer = reducer::Method::makeMethod(m_method, std::cout);

  std::vector<std::vector<reducer::Lit>> clausesVivi;
  reducer->run(problem, m_nbIteration, true, clausesVivi);

  ProblemManagerCnf *ret = new ProblemManagerCnf(
      pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
      pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

  std::vector<std::vector<Lit>> &clausesAfter = ret->getClauses();
  for (auto &cl : clausesVivi) {
    clausesAfter.push_back({});
    for (auto &l : cl)
      clausesAfter.back().push_back(Lit::makeLit(l.var(), l.sign()));
  }

  delete reducer;

  return ret;
}  // run
}  // namespace d4
