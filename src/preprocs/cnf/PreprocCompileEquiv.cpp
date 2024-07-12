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

  // get the cnf.
  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  std::vector<std::vector<bipe::Lit>> clauses;
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  unsigned limitNbClauses = pcnf.getClauses().size();

  // call the preprocessor to compute the bipartition.
  std::vector<bipe::Var> input;
  std::vector<bipe::Gate> gates;

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
