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

#include "PartialOrderHeuristicGiven.hpp"

#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief PartialOrderHeuristicGiven::PartialOrderHeuristicGiven
 * implementation.
 */
PartialOrderHeuristicGiven::PartialOrderHeuristicGiven(
    const OptionPartialOrderHeuristic &options, FormulaManager &om,
    std::ostream &out) {
  out << "c [PARTIAL ORDER] Fixed by the user\n";
  assert(om.getNbVariable() < options.givenOrder.size());
  m_order = options.givenOrder;
  m_scaleFactor = options.scaleFactor;
}  // constructor

/**
 * @brief Destructor.
 */
PartialOrderHeuristicGiven::~PartialOrderHeuristicGiven() {}  // destructor

/**
 * @brief PartialOrderHeuristicGiven::computeCutSet implementation.
 */
void PartialOrderHeuristicGiven::computeCutSet(std::vector<Var> &component,
                                               std::vector<Var> &cutSet) {
  if (!component.size()) return;

  unsigned min = m_order[component[0]];
  for (auto &v : component)
    if (m_order[v] < min) min = m_order[v];

  for (auto &v : component)
    if (m_order[v] == min) cutSet.push_back(v);
}  // computeCutSet
}  // namespace d4