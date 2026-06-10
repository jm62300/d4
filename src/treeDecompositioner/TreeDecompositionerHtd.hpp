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

#pragma once

#include "TreeDecompositioner.hpp"

namespace d4 {

class TreeDecompositionerHtd : public TreeDecompositioner {
 public:
  /**
   * @brief Compute a tree decomposition using the htd library (bucket
   * elimination with the library's default ordering). The algorithm is
   * restarted with different random tie-breaking while the budget is not
   * exhausted and the width keeps improving; the best decomposition is kept.
   *
   * @param graph is the graph we want to handle.
   * @param budget is a time in second the method is run.
   * @param seed is the seed used for the random number generator.
   * @param verbose controls the verbosity.
   *
   * @return a tree decomposition of graph.
   */
  TreeDecomp *constructTreeDecomposition(Graph &graph, unsigned budget,
                                         unsigned seed, bool verbose) override;
};

}  // namespace d4
