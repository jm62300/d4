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

#include "3rdParty/bipe/srcBipe/methods/Bipartition.hpp"

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
                                           std::vector<bipe::Gate> &gates,
                                           unsigned timeout) {
  std::vector<Var> protect, selected;
  if (pcnf.getSelectedVar().size())
    selected = pcnf.getSelectedVar();
  else
    for (unsigned i = 1; i <= pcnf.getNbVar(); i++)
      if (pcnf.getWeightLit(Lit::makeLitTrue(i)) ==
          pcnf.getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);

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
}  // computeBipartition

/**
 * @brief applyDistillation implementation.
 */
bool PreprocSharpEquiv::applyDistillation(
    std::vector<std::vector<Lit>> &clauses, std::vector<Lit> &units,
    std::vector<bool> &isUnit, unsigned nbVar,
    std::vector<std::vector<Lit>> &resClauses) {
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
      if (!isUnit[cl[0].var()])
        units.push_back(Lit::makeLit(cl[0].var(), cl[0].sign()));
      continue;
    }

    resClauses.push_back({});
    for (auto &l : cl)
      resClauses.back().push_back(Lit::makeLit(l.var(), l.sign()));
  }

  return initSize > resClauses.size();
}  // applyDistillation

/**
 * @brief applyElimination implementation.
 */
bool PreprocSharpEquiv::applyElimination(
    std::vector<std::vector<Lit>> &clauses, std::vector<Lit> &units,
    std::vector<bool> &isUnit, unsigned nbVar, std::vector<bipe::Var> &input,
    std::vector<eliminator::Gate> &dac,
    std::vector<eliminator::Lit> &eliminated,
    std::vector<std::vector<Lit>> &resClauses, unsigned limitNbClauses) {
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
  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  unsigned limitNbClauses = pcnf.getClauses().size();

  // call the preprocessor to compute the bipartition.
  std::vector<bipe::Var> input;
  std::vector<bipe::Gate> gates;
  computeBipartition(pcnf, units, input, gates, timeout / 2);

  // prepare the formula we will return.
  ProblemManagerCnf *ret = new ProblemManagerCnf(
      pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
      pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

  // Start by trying to reduce the formula using distillation.
  int nbInitialClause = (int)pcnf.getClauses().size();
  m_isRunningReducer = reducer::Method::makeMethod("combinaison", std::cout);
  applyDistillation(pcnf.getClauses(), units, isUnit, pcnf.getNbVar(),
                    ret->getClauses());
  std::cout << "c [PREPROC #EQUIV] First call, #remove clauses: "
            << nbInitialClause - (int)ret->getClauses().size() << "\n";

  // prepare the method to eliminate variables.
  eliminator::Eliminator el;
  m_isRunningEliminator = &el;
  std::vector<eliminator::Lit> eliminated;
  std::vector<eliminator::Gate> dac;
  expressDacInEliminatorFormat(gates, dac);

  PreprocManager::s_isRunning = this;
  signal(SIGALRM, [](int s) {
    if (PreprocManager::s_isRunning)
      ((PreprocSharpEquiv *)PreprocManager::s_isRunning)->interrupt();
  });
  alarm(timeout / 2);

  bool hasBeenModified = true;
  for (unsigned ite = 0;
       hasBeenModified && !m_isInterrupted && ite < m_nbIteration; ite++) {
    // elimination.
    hasBeenModified = applyElimination(ret->getClauses(), units, isUnit,
                                       pcnf.getNbVar(), input, dac, eliminated,
                                       ret->getClauses(), limitNbClauses);

    // reduction.
    hasBeenModified = hasBeenModified ||
                      applyDistillation(ret->getClauses(), units, isUnit,
                                        pcnf.getNbVar(), ret->getClauses());

    std::cout << "c [PREPROC #EQUIV] #iteration: " << ite << "/"
              << m_nbIteration << "\t#clause: " << ret->getClauses().size()
              << "\t#eliminated: " << eliminated.size() << "\n";
  }
  PreprocManager::s_isRunning = nullptr;

  unsigned nbUsedGate = 0;
  for (auto &g : dac)
    if (g.input.size() && g.type == eliminator::RM) nbUsedGate++;
  std::cout << "c [PREPROC #EQUIV] Number gates used: " << nbUsedGate << "\n";

  // get the 'unit literals'.
  for (auto &l : units) ret->getClauses().push_back({l});

  alarm(0);
  delete m_isRunningReducer;
  return ret;
}  // run

/**
 * @brief expressDacInEliminatorFormat implementation.
 */
void PreprocSharpEquiv::expressDacInEliminatorFormat(
    std::vector<bipe::Gate> &gates, std::vector<eliminator::Gate> &dac) {
  unsigned nbEquiv = 0, nbUnit = 0, nbXor = 0, nbOr = 0;

  for (auto &g : gates) {
    dac.push_back(eliminator::Gate());
    dac.back().output =
        eliminator::Lit::makeLit(g.output.var(), g.output.sign());

    switch (g.type) {
      case bipe::UNIT:
        dac.back().type = eliminator::UNIT;
        nbUnit++;
        break;
      case bipe::EQUIV:
        dac.back().type = eliminator::EQUIV;
        nbEquiv++;
        break;
      case bipe::AND:
        nbOr++;
        dac.back().type = eliminator::AND;
        break;
      case bipe::OR:
        nbOr++;
        dac.back().type = eliminator::OR;
        break;
      case bipe::XOR:
        nbXor++;
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

  std::cout << "c [PREPROC #EQUIV] #unit = " << nbUnit << "\t"
            << "#equiv = " << nbEquiv << "\t"
            << "#or = " << nbOr << "\t"
            << "#xor = " << nbXor << "\n";
}  // expressDacInEliminatorFormat

}  // namespace d4
