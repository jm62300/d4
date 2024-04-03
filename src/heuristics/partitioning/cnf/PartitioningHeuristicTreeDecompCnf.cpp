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

#include "PartitioningHeuristicTreeDecompCnf.hpp"

#include <ostream>

#include "src/hyperGraph/treeDecomposition/TreeDecomposition.hpp"

namespace d4 {

/**
 * @brief PartitioningHeuristicTreeDecompCnf::PartitioningHeuristicTreeDecompCnf
 * implementation.
 */
PartitioningHeuristicTreeDecompCnf::PartitioningHeuristicTreeDecompCnf(
    const OptionPartitioningHeuristic &options, SpecManagerCnf &om,
    WrapperSolver &s, std::ostream &out) {
  TreeDecomposition *decomp =
      TreeDecomposition::makeTreeDecomposition(options, PB_CNF, out);

  std::vector<std::vector<Var>> decomposition;
  decomp->computeDecomposition(decomposition, om);

  std::vector<Var> component;
  for (unsigned i = 1; i <= om.getNbVariable(); i++) component.push_back(i);
  decomp->checkDecomposition(decomposition, om, component);

  delete decomp;
}  // constructor

/**
 * @brief Destructor.
 */
PartitioningHeuristicTreeDecompCnf::~PartitioningHeuristicTreeDecompCnf() {
}  // destructor

}  // namespace d4
