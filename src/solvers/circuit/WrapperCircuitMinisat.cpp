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

#include "WrapperCircuitMinisat.hpp"

#include <cassert>
#include <iostream>
#include <typeinfo>

#include "src/problem/ProblemManager.hpp"
#include "src/utils/Translator.hpp"

namespace d4 {
using minisat::toInt;

/**
 * @brief WrapperCircuitMinisat::initSolver implementation.
 */
void WrapperCircuitMinisat::initSolver(const ProblemManager& p) {
  std::cout << "c [MINISAT CIRCUIT SOLVER] Init phase\n";

  if (p.getProblemInputType() != PB_CIRC)
    std::runtime_error("A circuit was expected here!");

  while ((unsigned)m_solver.nVars() <= p.getNbVar()) m_solver.newVar();
  m_model.resize(p.getNbVar() + 1, l_Undef);

  initCadical(p.getNbVar());

  std::vector<std::vector<Lit>> clauses;
  Translator::tseitinEncoding(p.getGates(), clauses);
  m_initClauses.reserve(clauses.size());

  for (auto& cl : clauses) {
    minisat::vec<minisat::Lit> lits;
    std::vector<int> intCl;
    for (auto& l : cl) {
      intCl.push_back(l.human());
      m_cadical.add(l.human());
      lits.push(minisat::mkLit(l.var(), l.sign()));
    }
    m_initClauses.push_back(intCl);
    m_cadical.add(0);
    m_solver.addClause(lits);
  }

  m_activeModel = false;
  m_needModel = false;
  setNeedModel(m_needModel);
  m_isInAssumption.resize(p.getNbVar() + 1, 0);
}  // initSolver

}  // namespace d4
