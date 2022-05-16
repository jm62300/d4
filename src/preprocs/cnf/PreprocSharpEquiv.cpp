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
  std::vector<bool> isAdded(pin->getNbVar() + 1, false);
  ws->getUnits(units);

  // get the cnf.
  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);

  // call the preprocessor to compute the bipartition.
  std::vector<Var> protect, selected;
  if (pin->getSelectedVar().size())
    selected = pin->getSelectedVar();
  else
    for (unsigned i = 1; i <= pin->getNbVar(); i++)
      if (pin->getWeightLit(Lit::makeLitTrue(i)) ==
          pin->getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);

  bipe::Problem pb(pin->getNbVar(), pin->getWeightLit(), selected, protect);
  Lit::rewrite<bipe::Lit>(
      pcnf.getClauses(), units, pb.getClauses(),
      [](unsigned var, bool sign) { return bipe::Lit::makeLit(var, sign); });

  bipe::Bipartition bp;
  std::vector<bipe::Var> input;
  std::vector<bipe::Gate> gates;
  std::cerr << "c [PREPROC #EQUIV] Bipartition is running ...\n";
  bool res = bp.run(pb, input, gates, false, "Glucose_bipe", 0, "OCC_ASC",
                    false, true, true, true, true, std::cout);
  m_isRunningBackbone = &bp;

  if (!res) {
    std::cerr << "c [PREPOC #EQUIV] We already checked that is SAT Oo\n";
    exit(-1);
  }

  // call the method to eliminate variables.
  eliminator::Eliminator el;
  std::vector<eliminator::Lit> eliminated;
  std::vector<std::vector<eliminator::Lit>> clausesAfterElim;
  std::vector<eliminator::Gate> dac;

  // prepare the problem for elimination.
  Lit::rewrite<eliminator::Lit>(pcnf.getClauses(), units, clausesAfterElim,
                                [](unsigned var, bool sign) {
                                  return eliminator::Lit::makeLit(var, sign);
                                });
  expressDacInEliminatorFormat(gates, dac);
  el.eliminateDac(pin->getNbVar(), clausesAfterElim, dac, eliminated);

  for (auto &l : units) isAdded[l.var()] = true;
  for (auto &l : eliminated)
    if (!isAdded[l.var()]) {
      units.push_back(Lit::makeLit(l.var(), l.sign()));
      isAdded[l.var()] = true;
    }

  if (!m_isInterrupted) {
    // apply the vivification and the occurrence elimination proccess.
    std::vector<std::vector<reducer::Lit>> fclauses;
    Lit::rewrite<eliminator::Lit, reducer::Lit>(
        clausesAfterElim, fclauses, [](eliminator::Lit l) {
          return reducer::Lit::makeLit(l.var(), l.sign());
        });

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
    for (auto &l : units) clausesAfter.push_back({l});

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

/**
 * @brief expressDacInEliminatorFormat implementation.
 */
void PreprocSharpEquiv::expressDacInEliminatorFormat(
    std::vector<bipe::Gate> &gates, std::vector<eliminator::Gate> &dac) {
  for (auto &g : gates) {
    dac.push_back(eliminator::Gate());
    dac.back().output =
        eliminator::Lit::makeLit(g.output.var(), g.output.sign());

    switch (g.type) {
      case bipe::UNIT:
        dac.back().type = eliminator::UNIT;
        break;
      case bipe::EQUIV:
        dac.back().type = eliminator::EQUIV;
        break;
      case bipe::AND:
        dac.back().type = eliminator::AND;
        break;
      case bipe::OR:
        dac.back().type = eliminator::OR;
        break;
      case bipe::XOR:
        dac.back().type = eliminator::XOR;
        break;
      default:
        std::cerr << "c This gate is not supported\n";
        exit(EXIT_FAILURE);
        break;
    }

    for (auto l : g.input)
      dac.back().input.push_back(eliminator::Lit::makeLit(l.var(), l.sign()));
  }
}  // expressDacInEliminatorFormat

}  // namespace d4
