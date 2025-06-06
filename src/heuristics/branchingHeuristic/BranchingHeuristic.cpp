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
#include "BranchingHeuristicHybridPartialClassic.hpp"
#include "BranchingHeuristicLargeArity.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief ListLit::ListLit implementation.
 */
ListLit::ListLit() {
  m_size = 0;
  m_array = NULL;
  m_listLitAllocator = NULL;
}  // constructor

/**
 * @brief ListLit::ListLit implementation.
 */
void ListLit::setListLit(const Lit *tab, int size,
                         ListLitAllocator *litListAllocator) {
  if (!size) {
    m_size = 0;
    m_array = NULL;
    m_listLitAllocator = NULL;
  } else {
    m_listLitAllocator = litListAllocator;
    m_array = m_listLitAllocator->askMemory(size);
    m_size = size;
    for (unsigned i = 0; i < size; i++) m_array[i] = tab[i];
  }
}  // constructor

/**
 * @brief ListLit::ListLit implementation.
 */
ListLit::~ListLit() {
  if (m_size && m_listLitAllocator)
    m_listLitAllocator->freeMemory(m_array, m_size);
}  // constructor

/**
 * @brief BranchingHeuristic::BranchingHeuristic implementation.
 */
BranchingHeuristic::BranchingHeuristic(const OptionBranchingHeuristic &options,
                                       ProblemManager *problem,
                                       FormulaManager *specs,
                                       ActivityManager &activityManager,
                                       PolarityManager &polarityManager,
                                       std::ostream &out) {
  out << "c [BRANCHING HEURISTIC]" << options << "\n";

  m_hVar =
      ScoringMethod::makeScoringMethod(options, *specs, activityManager, out);
  m_hPhase =
      PhaseHeuristic::makePhaseHeuristic(options, *specs, polarityManager, out);
  m_freqDecay = options.freqDecay;
  m_specs = specs;
  m_problem = problem;
  m_nbCall = 0;
  m_listLitAllocator = new ListLitAllocator();

  m_isDecisionVariable.resize(problem->getNbVar() + 1,
                              !problem->getNbSelectedVar());
  for (auto &v : problem->getSelectedVar()) {
    m_isDecisionVariable[v] = true;
  }
}  // constructor

/**
 * @brief BranchingHeuristic::~BranchingHeuristic implementation.
 *
 */
BranchingHeuristic::~BranchingHeuristic() {
  delete m_hVar;
  delete m_hPhase;
  delete m_listLitAllocator;
}  // destructor

/**
 * @brief BranchingHeuristic::makeBranchingHeuristic implementation.
 *
 */
BranchingHeuristic *BranchingHeuristic::makeBranchingHeuristic(
    const OptionBranchingHeuristic &options, ProblemManager *problem,
    FormulaManager *specs, ActivityManager &activityManager,
    PolarityManager &polarityManager, std::ostream &out) {
  if (problem->getNbSelectedVar()) {
    out << "c [MODE] Projected we can only use the classical heuristic\n";
    return new BranchingHeuristicClassic(options, problem, specs,
                                         activityManager, polarityManager, out);
  }

  out << "c [MODE] classic\n";
  switch (options.branchingHeuristicType) {
    case BRANCHING_CLASSIC:
      return new BranchingHeuristicClassic(
          options, problem, specs, activityManager, polarityManager, out);
    case BRANCHING_HYBRID_PARTIAL_CLASSIC:
      return new BranchingHeuristicHybridPartialClassic(
          options, problem, specs, activityManager, polarityManager, out);
    case BRANCHING_LARGE_ARITY:
      return new BranchingHeuristicLargeArity(
          options, problem, specs, activityManager, polarityManager, out);
  }

  throw(FactoryException("Cannot create a BranchingHeuristic", __FILE__,
                         __LINE__));
}  // makeBranchingHeuristic
}  // namespace d4