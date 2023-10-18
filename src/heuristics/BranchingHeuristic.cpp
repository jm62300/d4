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

#include "BranchingHeuristic.hpp"

namespace d4 {

/**
 * @brief BranchingHeuristic::BranchingHeuristic implementation.
 */
BranchingHeuristic::BranchingHeuristic(const OptionBranchingHeuristic &options,
                                       SpecManager *specs,
                                       WrapperSolver *solver,
                                       std::ostream &out) {
  m_hVar = ScoringMethod::makeScoringMethod(options, *specs, *solver, out);
  m_hPhase = PhaseHeuristic::makePhaseHeuristic(options, *specs, *solver, out);
  m_freqDecay = options.freqDecay;
  m_specs = specs;
  m_nbCall = 0;
}  // constructor

/**
 * @brief BranchingHeuristic::~BranchingHeuristic implementation.
 *
 */
BranchingHeuristic::~BranchingHeuristic() {
  delete m_hVar;
  delete m_hPhase;
}  // destructor

/**
 * @brief BranchingHeuristic::selectLitSet implementation.
 */
unsigned BranchingHeuristic::selectLitSet(std::vector<Var> &vars,
                                          std::vector<bool> &isDecisionVariable,
                                          Lit *lits) {
  m_nbCall++;

  // decay the variable weights.
  if (m_freqDecay && !(m_nbCall % m_freqDecay)) m_hVar->decayCountConflict();

  Var v = m_hVar->selectVariable(vars, *m_specs, isDecisionVariable);
  if (v == var_Undef) return 0;

  *lits = Lit::makeLit(v, m_hPhase->selectPhase(v));
  return 1;
}  // selectLitSet

}  // namespace d4