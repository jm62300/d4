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

#include "TreeDecompositionerFlowCutter.hpp"

#include <chrono>
#include <ctime>

#include "3rdParty/flowCutter/src/pace.h"

namespace d4 {

/**
 * @brief TreeDecompositionerFlowCutter::constructTreeDecomposition
 * implementation.
 */
TreeDecomp *TreeDecompositionerFlowCutter::constructTreeDecomposition(
    Graph &graph) {
  auto start = std::chrono::system_clock::now();

  const char *decomp =
      flowCutter::paceMain(graph.getNbNode(), graph.getEdge(), 11);
  // std::cout << "print the decomposition:\n" << decomp;

  auto end = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s" << std::endl;

  return NULL;
}  // constructTreeDecomposition
}  // namespace d4