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

#include "3rdParty/bipe/src/bipartition/methods/Backbone.hpp"
#include "3rdParty/bipe/src/bipartition/methods/Bipartition.hpp"
#include "3rdParty/bipe/src/bipartition/methods/DACircuit.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"

namespace d4 {

/**
 * The constructor.
 *
 * @param[in] nbIteration is the number of iteration we will do.
 * @param[in] out is the stream where are printed out the logs.
 */
PreprocSharpEquiv::PreprocSharpEquiv(int nbIteration, std::ostream &out) {
  m_nbIteration = nbIteration;
}  // constructor

/**
 * @brief computeBipartition implementation.
 */
bool PreprocSharpEquiv::computeBipartition(ProblemManagerCnf &pcnf,
                                           std::vector<Lit> &units,
                                           std::vector<bipe::Var> &input,
                                           std::vector<bipe::Var> &output,
                                           std::vector<bipe::Gate> &gates,
                                           const OptionPreprocManager &option) {
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

  std::vector<double> tmp(pcnf.getNbVar() + 1, 1.0);
  bipe::Problem pb(pcnf.getNbVar(), tmp, selected, protect);
  Lit::rewrite<bipe::Lit>(
      pcnf.getClauses(), units, pb.getClauses(),
      [](unsigned var, bool sign) { return bipe::Lit::makeLit(var, sign); });

  // Options:
  bipe::bipartition::OptionBackbone optionBackbone(false, 0, true, "glucose");
  bipe::bipartition::OptionDac optionDac(false, "glucose");

  std::string order = (option.ordered) ? "NATURAL_ORDER" : "OCC_ASC";
  bipe::bipartition::OptionBipartition optionBipartition(
      false, true, true, order, "glucose", 200, 5);

  bipe::bipartition::Bipartition b;
  bipe::Problem *formula = nullptr;

  s_isRunning = &b;
  std::cout << "c [PREPROC #EQUIV] Bipartition is running ...\n";

  // change the handler.
  void (*handler)(int) = [](int s) {
    if (PreprocManager::s_isRunning)
      ((bipe::bipartition::Method *)PreprocManager::s_isRunning)->interrupt();
  };
  signal(SIGALRM, handler);
  alarm(option.timeout);

  std::vector<std::vector<bool>> setOfModels;
  formula =
      b.simplifyBackbone(pb, optionBackbone, gates, std::cout, setOfModels);
  bool isSAT = !formula->isTriviallyUnsat();

  if (isSAT) {
    if (formula) {
      bipe::Problem *tmp = formula;
      if (option.ordered) b.setOrder(pcnf.getOrder());
      formula = b.simplifyDac(*tmp, optionDac, gates, std::cout, setOfModels);
      if (!formula)
        formula = tmp;
      else
        delete tmp;
    }

    if (!formula) {
      input = pb.getProjectedVar();
    } else if (!option.onlyUseGates) {
      std::vector<std::vector<bipe::Var>> symGroup;
      bool res = b.run(*formula, input, gates, optionBipartition, symGroup,
                       setOfModels, std::cout);

      if (!res) {
        std::cout << "c [PREPOC BACKBONE] The preproc has been stopped before "
                     "the end\n";
      }
    }

    if (option.onlyUseGates) {
      std::vector<bool> marked(pb.getNbVar() + 1, false);
      for (auto &g : gates) marked[g.output.var()] = true;
      for (auto v : pb.getProjectedVar())
        if (!marked[v]) input.push_back(v);
    }

    // put the remaining variable into the output set.
    std::vector<bool> marked(pb.getNbVar() + 1, false);
    for (auto &v : input) marked[v] = true;
    for (unsigned i = 1; i < pb.getNbVar() + 1; i++)
      if (!marked[i]) output.push_back(i);
    assert(output.size() + input.size() == pb.getNbVar());
    alarm(0);
  }

  delete formula;
  s_isRunning = NULL;
  std::cout << "c [PREPROC #EQUIV] Terminated with status: " << isSAT << "\n";
  return isSAT;
}  // computeBipartition

/**
 * @brief PreprocSharpEquiv::~PreprocSharpEquiv implementation.
 */
PreprocSharpEquiv::~PreprocSharpEquiv() {}  // destructor

/**
 * @brief PreprocSharpEquiv::fixGatesModuloOrder implementation.
 */
void PreprocSharpEquiv::fixGatesModuloOrder(std::vector<bipe::Gate> &gates,
                                            std::vector<bipe::Var> &input,
                                            std::vector<bipe::Var> &output,
                                            std::vector<unsigned> &order) {
  // remove gates they do not follow the order.
  std::vector<bool> moved(order.size() + 1, false);
  std::vector<bipe::Gate> reduceGates;
  for (auto &g : gates) {
    bool isOrdered = true;

    for (auto &l : g.input)
      if (order[g.output.var()] < order[l.var()]) {
        isOrdered = false;
        break;
      }

    if (isOrdered)
      reduceGates.push_back(g);
    else {
      input.push_back(g.output.var());
      moved[g.output.var()] = true;
    }
  }

  gates = reduceGates;

  unsigned i, j;
  for (i = j = 0; i < output.size(); i++)
    if (!moved[output[i]]) output[j++] = output[i];
  moved.resize(j);
}  // fixGatesModuloOrder

/**
 * @brief PreprocSharpEquiv::run implementation.
 */
ProblemManager *PreprocSharpEquiv::run(ProblemManager *pin,
                                       const OptionPreprocManager &option) {
  std::cout << "c [PREPROC #EQUIV] Start\n";

  std::vector<bool> isUnit(pin->getNbVar() + 1, false);

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
      else
        protect.push_back(i);

  std::vector<double> tmp(pin->getNbVar() + 1, 1.0);
  bipe::Problem pb(pin->getNbVar(), tmp, selected, protect);

  ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf &>(*pin);
  std::vector<std::vector<bipe::Lit>> &clauses = pb.getClauses();
  for (auto &cl : pcnf.getClauses()) {
    clauses.push_back({});
    for (auto l : cl)
      clauses.back().push_back(bipe::Lit::makeLit(l.var(), l.sign()));
  }

  unsigned limitNbClauses = pcnf.getClauses().size();

  // call the preprocessor to compute the bipartition.
  std::vector<bipe::Var> input, output;
  std::vector<bipe::Gate> gates;
  std::vector<Lit> units;
  bool isSat = computeBipartition(pcnf, units, input, output, gates, option);

  std::cout << "c [PREPROC] Number of output detected: " << output.size()
            << '\n';
  std::cout << "c [PREPROC] ";
  for (auto &v : output) std::cout << v << ' ';
  std::cout << '\n';

  if (option.ordered)
    fixGatesModuloOrder(gates, input, output, pin->getOrder());
  if (!isSat) return pin->getUnsatProblem();

  // create the problem from the reducer side.
  bipe::eliminator::Eliminator e;
  bipe::reducer::Method *rm =
      bipe::reducer::Method::makeMethod("combinaison", std::cout);

  std::vector<bool> markedAsUnit(pcnf.getNbVar() + 1, false);
  for (auto &g : gates)
    markedAsUnit[g.output.var()] = g.type == bipe::TypeGate::UNIT;

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

  // print out the eliminated variables.
  unsigned cpt = 0;
  std::cout << "c Removed variables: ";
  for (auto &l : eliminated)
    if (!markedAsUnit[l.var()]) {
      std::cout << l.var() << ' ';
      cpt++;
    }
  std::cout << "0\n";
  std::cout << "c Number of defined variables removed (not unit): " << cpt
            << '\n';

  delete rm;
  return ret;
}  // run

}  // namespace d4
