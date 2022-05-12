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

#include "PreprocSharpEquiv.hpp"

#include "3rdParty/bipe/srcBipe/methods/Bipartition.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {

/**
 * The constructor.
 *
 * @param[in] vm, the options used (solver).
 */
PreprocSharpEquiv::PreprocSharpEquiv(po::variables_map &vm, std::string &method,
                                     int nbIteration, std::ostream &out) {
  ws = WrapperSolver::makeWrapperSolverPreproc(vm, out);
  m_method = method;
  m_nbIteration = nbIteration;
}  // constructor

/**
   Destructor.
 */
PreprocSharpEquiv::~PreprocSharpEquiv() { delete ws; }  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocSharpEquiv::run(ProblemManager *pin,
                                       LastBreathPreproc &lastBreath) {
  std::cout << "c [PREPROC #EQUIV] Start\n";
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
  bipe::Bipartition bp;

  std::vector<bipe::Var> input;
  std::vector<bipe::Gate> gates;
  std::cerr << "c [PREPROC #EQUIV] Bipartition is running ...\n";
  bool res = bp.run(pb, input, gates, false, "Glucose_bipe", 0, "OCC_ASC", true,
                    true, true, true, true, std::cout);
  m_isRunningBackbone = &bp;

  if (!res) {
    std::cerr << "c [PREPOC #EQUIV] We already checked that is SAT Oo\n";
    exit(-1);
  }

  // the list of unit literals.
  units.clear();
  for (auto g : gates) {
    if (g.type == bipe::UNIT)
      units.push_back(Lit::makeLit(g.output.var(), g.output.sign()));
  }

  if (!m_isInterrupted) {
    // apply the vivification and the occurrence elimination proccess.
    std::vector<std::vector<reducer::Lit>> fclauses;
    for (auto l : units)
      fclauses.push_back({reducer::Lit::makeLit(l.var(), l.sign())});
    for (auto &cl : pcnf.getClauses()) {
      fclauses.push_back({});
      for (auto l : cl)
        fclauses.back().push_back(reducer::Lit::makeLit(l.var(), l.sign()));
    }

    // create the problem from the reducer side.
    reducer::Problem problem(fclauses, pcnf.getNbVar(), std::cout, false);
    m_isRunningReducer = reducer::Method::makeMethod("combinaison", std::cout);

    std::vector<std::vector<reducer::Lit>> clausesVivi;
    m_isRunningReducer->run(problem, m_nbIteration, true, clausesVivi);

    ProblemManagerCnf *ret = new ProblemManagerCnf(
        pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
        pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

    std::vector<std::vector<Lit>> &clausesAfter = ret->getClauses();
    for (auto &cl : clausesVivi) {
      clausesAfter.push_back({});
      for (auto &l : cl)
        clausesAfter.back().push_back(Lit::makeLit(l.var(), l.sign()));
    }

    reducer::Method *rm = m_isRunningReducer;
    m_isRunningReducer = NULL;
    m_isRunningBackbone = NULL;
    delete rm;
    return ret;
  } else {
    m_isRunningBackbone = NULL;
    return pin->getConditionedFormula(units);
  }
}  // run
}  // namespace d4
