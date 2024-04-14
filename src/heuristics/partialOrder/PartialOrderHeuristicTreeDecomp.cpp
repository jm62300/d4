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

#include "PartialOrderHeuristicTreeDecomp.hpp"

#include "cnf/PartialOrderHeuristicTreeDecompCnf.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief PartialOrderHeuristicTreeDecomp::makePartitioningTreeDecomp
 * implementation.
 */
PartialOrderHeuristicTreeDecomp *
PartialOrderHeuristicTreeDecomp::makePartitioningTreeDecomp(
    const OptionPartialOrderHeuristic &options, SpecManager &om,
    WrapperSolver &s, std::ostream &out) {
  switch (om.getProblemInputType()) {
    case PB_CNF:
      return new PartialOrderHeuristicTreeDecompCnf(
          options, dynamic_cast<SpecManagerCnf &>(om), s, out);
    default:
      throw(FactoryException("Cannot create a Partitioning Heuristic", __FILE__,
                             __LINE__));
  }
}  // makePartitioningHeuristicStatic

/**
 * @brief PartialOrderHeuristicTreeDecomp::computeCutSet implementation.
 */
void PartialOrderHeuristicTreeDecomp::computeCutSet(std::vector<Var> &component,
                                                    std::vector<Var> &cutSet) {
  if (!component.size()) return;

  unsigned min = m_topologicalOrder[component[0]];
  for (auto &v : component)
    if (m_topologicalOrder[v] < min) min = m_topologicalOrder[v];

  for (auto &v : component)
    if (m_topologicalOrder[v] == min) cutSet.push_back(v);
}  // computeCutSet

}  // namespace d4