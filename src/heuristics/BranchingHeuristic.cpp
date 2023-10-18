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

#include "BranchingHeuristicClassic.hpp"
#include "BranchingHeuristicLargeArity.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief BranchingHeuristic::BranchingHeuristic implementation.
 */
BranchingHeuristic::BranchingHeuristic(const OptionBranchingHeuristic &options,
                                       SpecManager *specs,
                                       WrapperSolver *solver,
                                       std::ostream &out) {
  out << "c [BRANCHING HEURISTIC]" << options << "\n";

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
 * @brief BranchingHeuristic::makeBranchingHeuristic implementation.
 */
BranchingHeuristic *BranchingHeuristic::makeBranchingHeuristic(
    const OptionBranchingHeuristic &options, SpecManager *m_specs,
    WrapperSolver *m_solver, std::ostream &out) {
  if (options.branchingHeuristicType == BRANCHING_CLASSIC)
    return new BranchingHeuristicClassic(options, m_specs, m_solver, out);
  if (options.branchingHeuristicType == BRANCHING_LARGE_ARITY)
    return new BranchingHeuristicLargeArity(options, m_specs, m_solver, out);

  throw(FactoryException("Cannot create a BranchingHeuristic", __FILE__,
                         __LINE__));
}  // makeBranchingHeuristic

}  // namespace d4