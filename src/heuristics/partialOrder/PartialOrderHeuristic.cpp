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
#include "PartialOrderHeuristic.hpp"

#include "PartialOrderHeuristicGiven.hpp"
#include "PartialOrderHeuristicNone.hpp"
#include "PartialOrderHeuristicTreeDecomp.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief PartialOrderHeuristic::makePartialOrderingHeuristic implementation.
 */
PartialOrderHeuristic* PartialOrderHeuristic::makePartialOrderingHeuristic(
    const OptionPartialOrderHeuristic& options, FormulaManager& s,
    std::ostream& out) {
  switch (options.partialOrderMethod) {
    case PARTIAL_ORDER_NONE:
      out << "c [PARTIAL ORDER HEURISTIC] None\n";
      return new PartialOrderHeuristicNone();
    case PARTIAL_ORDER_TREE_DECOMPOSITION:
      out << "c [PARTIAL ORDER HEURISTIC] Tree decomposition\n";
      return new PartialOrderHeuristicTreeDecomp(options, s, out);
    case PARTIAL_ORDER_GIVEN:
      out << "c [PARTIAL ORDER HEURISTIC] Given\n";
      return new PartialOrderHeuristicGiven(options, s, out);
  }

  throw(FactoryException("Cannot create a PartialOrderHeuristic", __FILE__,
                         __LINE__));
}  // makePartialOrderingHeuristic

}  // namespace d4
