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

#include "WrapperCircuitGlucose.hpp"

#include <iostream>
#include <typeinfo>
#include <vector>

#include "3rdParty/glucose-3.0/core/Solver.h"
#include "3rdParty/glucose-3.0/core/SolverTypes.h"
#include "3rdParty/glucose-3.0/mtl/Vec.h"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/utils/Translator.hpp"

namespace d4 {
/**
 * @brief WrapperCircuitGlucose::initSolver implementation.
 */
void WrapperCircuitGlucose::initSolver(const ProblemManager& p) {
  std::cout << "c [GLUCOSE CIRCUIT SOLVER] Init phase\n";

  if (p.getProblemInputType() != PB_CIRC)
    std::runtime_error("A circuit was expected here!");

  // say to the solver we have pcnf.getNbVar() variables.
  while ((unsigned)m_solver.nVars() <= p.getNbVar()) m_solver.newVar();
  m_model.resize(p.getNbVar() + 1, l_Undef);

  std::vector<std::vector<Lit>> clauses;

  Translator::tseitinEncoding(p.getGates(), clauses);

  for (auto& cl : clauses) {
    Glucose::vec<Glucose::Lit> lits;
    for (auto& l : cl) lits.push(Glucose::mkLit(l.var(), l.sign()));
    m_solver.addClause(lits);
  }

  m_activeModel = false;
  m_needModel = false;
  setNeedModel(m_needModel);
  m_isInAssumption.resize(p.getNbVar() + 1, 0);
}  // initSolver

}  // namespace d4