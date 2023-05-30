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

#include <csignal>

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
 * @brief computeBipartition implementation.
 */
void PreprocSharpEquiv::computeBipartition(ProblemManagerCnf &pcnf,
                                           std::vector<Lit> &units,
                                           std::vector<bipe::Var> &input,
                                           std::vector<bipe::Var> &output,
                                           std::vector<bipe::Gate> &gates,
                                           unsigned timeout) {
#if 0
  std::vector<Var> protect, selected;
  if (pcnf.getSelectedVar().size())
    selected = pcnf.getSelectedVar();
  else
    for (unsigned i = 1; i <= pcnf.getNbVar(); i++)
      if (pcnf.getWeightLit(Lit::makeLitTrue(i)) ==
          pcnf.getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);
      else
        protect.push_back(i);

  bipe::Problem pb(pcnf.getNbVar(), pcnf.getWeightLit(), selected, protect);
  Lit::rewrite<bipe::Lit>(
      pcnf.getClauses(), units, pb.getClauses(),
      [](unsigned var, bool sign) { return bipe::Lit::makeLit(var, sign); });

  bipe::Bipartition bp;
  std::cout << "c [PREPROC #EQUIV] Bipartition is running ...\n";
  PreprocManager::s_isRunning = &bp;
  bool res = true;

  signal(SIGALRM, [](int s) {
    if (PreprocManager::s_isRunning)
      ((bipe::Bipartition *)PreprocManager::s_isRunning)->interrupt();
  });
  alarm(timeout);

  res = bp.run(pb, input, gates, false, "Glucose_bipe", 0, "OCC_ASC", true,
               true, true, true, true, std::cout);
  PreprocManager::s_isRunning = nullptr;
  std::cout << "c [PREPROC #EQUIV] ... done\n";

  if (!res) {
    std::cerr << "c [PREPOC #EQUIV] We already checked that is SAT Oo\n";
    exit(-1);
  }
#endif
}  // computeBipartition

/**
 * @brief applyDistillation implementation.
 */
bool PreprocSharpEquiv::applyDistillation(
    std::vector<std::vector<Lit>> &clauses, std::vector<Lit> &units,
    std::vector<bool> &isUnit, unsigned nbVar,
    std::vector<std::vector<Lit>> &resClauses) {
#if 0
  // the clauses that will given to the distillation process.
  unsigned initSize = clauses.size();
  std::vector<std::vector<reducer::Lit>> dclauses, resDist;

  // prepare the problem for distillation.
  Lit::rewrite<reducer::Lit>(
      clauses, units, dclauses,
      [](unsigned var, bool sign) { return reducer::Lit::makeLit(var, sign); });

  reducer::Problem problem(dclauses, nbVar, std::cout, false);
  m_isRunningReducer->run(problem, 1, false, resDist);

  // rewrite in the main d4 format.
  resClauses.clear();
  for (auto &cl : resDist) {
    if (!cl.size()) continue;
    if (cl.size() == 1) {
      if (!isUnit[cl[0].var()]) {
        units.push_back(Lit::makeLit(cl[0].var(), cl[0].sign()));
        isUnit[cl[0].var()] = true;
      }
      continue;
    }

    resClauses.push_back({});
    for (auto &l : cl)
      resClauses.back().push_back(Lit::makeLit(l.var(), l.sign()));
  }

  return initSize > resClauses.size();
#endif
  return true;
}  // applyDistillation

/**
 * @brief applyElimination implementation.
 */
bool PreprocSharpEquiv::applyElimination(
    std::vector<std::vector<Lit>> &clauses, std::vector<Lit> &units,
    std::vector<bool> &isUnit, unsigned nbVar, std::vector<bipe::Var> &input,
    std::vector<bipe::Gate> &dac, std::vector<bipe::Lit> &eliminated,
    std::vector<std::vector<Lit>> &resClauses, unsigned limitNbClauses) {
#if 0
  // prepare the clause for the elimination method.
  std::vector<std::vector<eliminator::Lit>> clausesAfterElim;

  Lit::rewrite<eliminator::Lit>(clauses, units, clausesAfterElim,
                                [](unsigned var, bool sign) {
                                  return eliminator::Lit::makeLit(var, sign);
                                });

  unsigned initElim = eliminated.size();
  m_isRunningEliminator->eliminate(nbVar, clausesAfterElim, input, dac,
                                   eliminated, false, limitNbClauses);

  resClauses.clear();
  for (auto &cl : clausesAfterElim) {
    if (!cl.size()) continue;
    if (cl.size() == 1) {
      if (!isUnit[cl[0].var()])
        units.push_back(Lit::makeLit(cl[0].var(), cl[0].sign()));
      continue;
    }

    resClauses.push_back({});
    for (auto &l : cl)
      resClauses.back().push_back(Lit::makeLit(l.var(), l.sign()));
  }

  for (unsigned i = initElim; i < eliminated.size(); i++) {
    if (isUnit[eliminated[i].var()]) continue;
    isUnit[eliminated[i].var()] = true;
    units.push_back(Lit::makeLit(eliminated[i].var(), eliminated[i].sign()));
  }

  return initElim < eliminated.size();
#endif
  return true;
}  // applyElimination

/**
 * @brief Destroy the Preproc Sharp Equiv:: Preproc Sharp Equiv object
 */
PreprocSharpEquiv::~PreprocSharpEquiv() { delete ws; }  // destructor

/**
 * @brief The preprocessing itself.
 * @param[out] p, the problem we want to preprocess.
 * @param[out] lastBreath gives information about the way the    preproc sees
 * the problem.
 */
ProblemManager *PreprocSharpEquiv::run(ProblemManager *pin,
                                       LastBreathPreproc &lastBreath,
                                       unsigned timeout) {
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

  std::vector<bool> isUnit(pin->getNbVar() + 1, false);
  for (auto &l : units) isUnit[l.var()] = true;

  // get the cnf.
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

  unsigned limitNbClauses = pcnf.getClauses().size();

  // call the preprocessor to compute the bipartition.
  std::vector<bipe::Var> input, output;
  std::vector<bipe::Gate> gates;
  computeBipartition(pcnf, units, input, output, gates, timeout);

  // create the problem from the reducer side.
  bipe::eliminator::Eliminator e;
  bipe::reducer::Method *rm =
      bipe::reducer::Method::makeMethod("combinaison", std::cout);

  // the reduction + elimination + reduction phase.
  rm->run(pin->getNbVar(), clauses, 5, true, clauses);
  std::vector<bipe::Lit> eliminated;
  e.eliminate(pin->getNbVar(), clauses, input, gates, eliminated, false,
              limitNbClauses);
  rm->run(pin->getNbVar(), clauses, 5, true, clauses);

  // the problem we return.
  ProblemManagerCnf *ret = new ProblemManagerCnf(
      pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
      pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

  // transfer the clauses.
  std::vector<std::vector<Lit>> &clausesAfter = ret->getClauses();
  for (auto &cl : clauses) {
    clausesAfter.push_back({});
    for (auto &l : cl)
      clausesAfter.back().push_back(Lit::makeLit(l.var(), l.sign()));
  }

  // to be sure to expel the removed variables.
  for (auto &l : eliminated)
    clausesAfter.push_back({Lit::makeLit(l.var(), l.sign())});

  delete rm;
  return ret;
}  // run

}  // namespace d4
