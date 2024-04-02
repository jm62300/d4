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
#include "PartitioningHeuristic.hpp"

#include "PartitioningHeuristicNone.hpp"
#include "PartitioningHeuristicTreeDecomp.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief PartitioningHeuristic::makePartitioningHeuristic implementation.
 */
PartitioningHeuristic *PartitioningHeuristic::makePartitioningHeuristic(
    const OptionPartitioningHeuristic &options, SpecManager &s,
    WrapperSolver &ws, std::ostream &out) {
  out << "c [PARTITIONING HEURISTIC]" << options << "\n";

  switch (options.partitioningMethod) {
    case PARTITIONING_NONE:
      return new PartitioningHeuristicNone();
    case PARTITIONING_TREE_DECOMP: {
      return PartitioningHeuristicTreeDecomp::makePartitioningTreeDecomp(
          options, s, ws, out);
    }
  }

  throw(FactoryException("Cannot create a PartitioningHeuristic", __FILE__,
                         __LINE__));
}  // makePartitioningHeuristic

}  // namespace d4
