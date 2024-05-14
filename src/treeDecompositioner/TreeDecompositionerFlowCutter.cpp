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
#include <iostream>
#include <sstream>

#include "3rdParty/flowCutter/src/pace.h"

namespace d4 {

/**
 * @brief TreeDecompositionerFlowCutter::constructTreeDecomposition
 * implementation.
 */
TreeDecomp *TreeDecompositionerFlowCutter::constructTreeDecomposition(
    Graph &graph) {
  TreeDecomp *ret = NULL;

  // compute the tree decomposition using flow cutter.
  auto start = std::chrono::system_clock::now();
  const char *decomp =
      flowCutter::paceMain(graph.getNbNode(), graph.getEdge(), 11);
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "c [FLOW-CUTTER] Elapsed time: " << elapsed_seconds.count()
            << "s" << std::endl;

  if (!decomp)  // cannot find a decomposition (in the given time).
  {
    std::vector<Var> vars;
    std::vector<bool> marked;

    for (auto &e : graph.getEdge()) {
      if (e.first >= marked.size() || !marked[e.first]) {
        marked.resize(e.first + 1, false);
        marked[e.first] = true;
        vars.push_back(e.first);
      }

      if (e.second >= marked.size() || !marked[e.second]) {
        marked.resize(e.second + 1, false);
        marked[e.second] = true;
        vars.push_back(e.second);
      }
    }

    ret = new TreeDecomp(vars, std::vector<TreeDecomp *>());
  } else {
    // parse the decomposition from the result returned by flow-cutter.
    std::istringstream f(decomp);
    std::string line;
    std::vector<TreeDecomp *> setOfTrees;
    std::vector<std::pair<unsigned, unsigned>> edges;

    while (std::getline(f, line)) {
      if (line.size() == 0) continue;
      if (line[0] == 's') continue;

      if (line[0] == 'b') {
        unsigned i = 1;
        while (i < line.size() && line[i] == ' ') i++;
        assert(i < line.size());

        // parse the index number
        unsigned idx = 0;
        while (line[i] != ' ' && i < line.size()) {
          idx = idx * 10 + (line[i] - '0');
          i++;
        }
        idx--;

        while (i < line.size() && line[i] == ' ') i++;

        // parse the variables.
        std::vector<Var> vars;
        while (i < line.size() && line[i] != '\n') {
          unsigned v = 0;

          while (line[i] != ' ' && i < line.size()) {
            v = v * 10 + (line[i] - '0');
            i++;
          }

          while (i < line.size() && line[i] == ' ') i++;

          vars.push_back(v);
        }

        assert(idx == setOfTrees.size());
        setOfTrees.push_back(new TreeDecomp(vars, std::vector<TreeDecomp *>()));
        nbParents.push_back(0);
      } else {
        unsigned e1 = 0, e2 = 0, i = 0;
        while (line[i] != ' ' && i < line.size()) {
          e1 = e1 * 10 + (line[i] - '0');
          i++;
        }
        while (i < line.size() && line[i] == ' ') i++;
        assert(i < line.size());

        e1--;

        while (line[i] != ' ' && i < line.size()) {
          e2 = e2 * 10 + (line[i] - '0');
          i++;
        }
        e2--;

        edges.push_back(std::make_pair(e1, e2));
      }
    }

    // create the tree from the edges.

    // select the root.

    ret = setOfTrees[0];
  }

  assert(ret);
  return ret;
}  // constructTreeDecomposition
}  // namespace d4