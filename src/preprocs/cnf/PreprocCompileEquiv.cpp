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
#include "PreprocCompileEquiv.hpp"

#include <csignal>

#include "3rdParty/bipe/src/bipartition/methods/Backbone.hpp"
#include "3rdParty/bipe/src/bipartition/methods/Bipartition.hpp"
#include "3rdParty/bipe/src/bipartition/methods/DACircuit.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"

namespace d4 {

/**
 * @brief PreprocCompileEquiv::PreprocCompileEquiv implementation.
 */
PreprocCompileEquiv::PreprocCompileEquiv(int nbIteration, std::ostream &out) {
  m_nbIteration = nbIteration;
}  // constructor

/**
 * @brief PreprocCompileEquiv::~PreprocCompileEquiv implementation.
 */
PreprocCompileEquiv::~PreprocCompileEquiv() {}  // destructor

/**
 * @brief PreprocCompileEquiv::run implementation.
 */
ProblemManager *PreprocCompileEquiv::run(ProblemManager *pin,
                                         const OptionPreprocManager &option) {
  std::cout << "c [PREPROC COMPILE-EQUIV] Start\n";

  std::vector<bool> isUnit(pin->getNbVar() + 1, false);

  // compute the backbone.
  std::vector<Var> protect, selected;
  if (pin->getSelectedVar().size())
    selected = pin->getSelectedVar();
  else
    for (unsigned i = 1; i <= pin->getNbVar(); i++)
      if (pin->getWeightLit(Lit::makeLitTrue(i)) ==
          pin->getWeightLit(Lit::makeLitFalse(i)))
        selected.push_back(i);

  std::vector<double> tmp(pin->getNbVar() + 1, 1.0);
  bipe::Problem pb(pin->getNbVar(), tmp, selected, protect);
  std::vector<std::vector<bipe::Lit>> &clauses = pb.getClauses();

  // get the cnf.
  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  unsigned limitNbClauses = pcnf.getClauses().size();

  // call the preprocessor to compute the backbone.
  bipe::bipartition::Method bb;
  std::vector<bipe::Gate> gates;
  std::vector<std::vector<bool>> setOfModels;

  std::cerr << "c [PREPOC BACKBONE] Is running for at most " << option.timeout
            << " seconds\n";

  PreprocManager::s_isRunning = &bb;

  // change the handler.
  void (*handler)(int) = [](int s) {
    if (PreprocManager::s_isRunning)
      ((bipe::bipartition::Method *)PreprocManager::s_isRunning)->interrupt();
  };
  signal(SIGALRM, handler);
  alarm(option.timeout);

  bipe::bipartition::OptionBackbone optionBackbone(false, 0, true, "glucose");
  bipe::Problem *pbTmp =
      bb.simplifyBackbone(pb, optionBackbone, gates, std::cout, setOfModels);
  s_isRunning = nullptr;

  if (!pbTmp) {
    std::cout
        << "c [PREPOC BACKBONE] The preproc has been stopped before the end\n";
    return pin;
  }

  if (pbTmp->isTriviallyUnsat()) return pin->getUnsatProblem();

  // the list of unit literals.
  for (auto g : gates)
    clauses.push_back({bipe::Lit::makeLit(g.output.var(), g.output.sign())});

  // get the bipartition.
  std::vector<bipe::Var> input;
  for (auto &v : pin->getSelectedVar()) input.push_back(v);

  // create the problem from the reducer side.
  bipe::eliminator::Eliminator e;
  e.setStrongElim(option.strongElim);

  bipe::reducer::Method *rm =
      bipe::reducer::Method::makeMethod("combinaison", std::cout);

  // the reduction + elimination + reduction phase.
  rm->run(pin->getNbVar(), clauses, 10, false, clauses);
  std::vector<bipe::Lit> eliminated;
  unsigned previousSize, runNumber = 1;
  do {
    std::cout << "c [PREPROC #EQUIV] Run number: " << runNumber++ << '\n';
    previousSize = eliminated.size();
    e.eliminate(pin->getNbVar(), clauses, input, gates, eliminated, false,
                limitNbClauses);

    rm->run(pin->getNbVar(), clauses, 10, false, clauses);
  } while (eliminated.size() != previousSize);

  // the problem we return.
  ProblemManagerCnf *ret = new ProblemManagerCnf(
      pin->getNbVar(), pin->getWeightLit(), pin->getWeightVar(),
      pin->getSelectedVar(), pin->getMaxVar(), pin->getIndVar());

  // sort the clauses regarding their size.
  std::sort(
      clauses.begin(), clauses.end(),
      [](const std::vector<bipe::Lit> &a, const std::vector<bipe::Lit> &b) {
        return a.size() < b.size();
      });

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
