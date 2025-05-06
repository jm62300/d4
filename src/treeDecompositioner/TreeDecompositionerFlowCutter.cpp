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
 * @brief TreeDecompositionerFlowCutter::getCenterGraph implementation.
 */
unsigned TreeDecompositionerFlowCutter::getCenterGraph(
    const std::vector<std::vector<unsigned>> &graph) {
  std::vector<unsigned> degree, degreeOne, nextDegreeOne;
  std::vector<bool> removed(graph.size() + 1, false);

  for (unsigned i = 0; i < graph.size(); i++) {
    degree.push_back(graph[i].size());
    if (degree[i] == 1) degreeOne.push_back(i);
  }

  while (degreeOne.size()) {
    // 'remove' nodes with degree one.
    nextDegreeOne.clear();

    for (auto n : degreeOne) {
      assert(!removed[n]);
      removed[n] = true;

      for (auto m : graph[n]) {
        if (removed[m]) continue;

        degree[m]--;
        if (!degree[m]) return m;
        if (degree[m] == 1) nextDegreeOne.push_back(m);
      }

      degreeOne = nextDegreeOne;
    }
  }

  return graph.size();
}  // getCenterGraph

/**
 * @brief TreeDecompositionerFlowCutter::makeTreeFromGraph implementation.
 */
void TreeDecompositionerFlowCutter::makeTreeFromGraph(
    const std::vector<std::vector<unsigned>> &graph, unsigned center,
    std::vector<TreeDecomp *> &setOfTrees, std::vector<bool> &marked) {
  marked[center] = true;

  for (auto &n : graph[center]) {
    if (marked[n]) continue;
    setOfTrees[center]->getSons().push_back(setOfTrees[n]);
    makeTreeFromGraph(graph, n, setOfTrees, marked);
  }
}  // makeTreeFromGraph

/**
 * @brief TreeDecompositionerFlowCutter::constructTreeDecomposition
 * implementation.
 */
TreeDecomp *TreeDecompositionerFlowCutter::constructTreeDecomposition(
    Graph &graph, unsigned budget, unsigned seed, bool verbose) {
  TreeDecomp *ret = NULL;

  // compute the tree decomposition using flow cutter.
  auto start = std::chrono::system_clock::now();
  const char *decomp = NULL;

  decomp = flowCutter::paceMain(graph.getNbNode(), graph.getEdge(), 29, budget,
                                true, seed, verbose);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  if (verbose)
    std::cout << "c [FLOW-CUTTER] Elapsed time: " << elapsed_seconds.count()
              << "s" << std::endl;

  if (!decomp)  // cannot find a decomposition (in the given time).
  {
    std::cout << "c [FLOW-CUTTER] Cannot find a decomposition\n";
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
    std::vector<std::vector<unsigned>> edges;

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
        edges.push_back(std::vector<unsigned>());
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

        assert(e1 < edges.size() && e2 < edges.size());
        edges[e1].push_back(e2);
        edges[e2].push_back(e1);
      }
    }

    if (setOfTrees.size() == 1)
      ret = setOfTrees[0];
    else {
      // create the tree from the edges.
      std::vector<bool> marked(edges.size() + 1, false);
      unsigned center = getCenterGraph(edges);
      assert(center < edges.size());
      makeTreeFromGraph(edges, center, setOfTrees, marked);

      // select the root.
      ret = setOfTrees[center];
    }
    delete[] decomp;
  }

  assert(ret);
  return ret;
}  // constructTreeDecomposition
}  // namespace d4