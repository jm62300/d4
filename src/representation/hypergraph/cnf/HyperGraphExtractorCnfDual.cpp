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

#include "HyperGraphExtractorCnfDual.hpp"

#include "src/formulaManager/cnf/CnfManager.hpp"

namespace d4 {

/**
 * @brief HyperGraphExtractorCnfDual::~HyperGraphExtractorCnfDual
 * implementation.
 */
HyperGraphExtractorCnfDual::~HyperGraphExtractorCnfDual() {
  if (m_data) delete[] m_data;
}  // destructor

/**
 * @brief HyperGraphExtractorCnfDual::constructHyperGraph implementation.
 */
InfoHyperGraph HyperGraphExtractorCnfDual::constructHyperGraph(
    FormulaManager &om, std::vector<Var> &component, HyperGraph &hypergraph) {
  // cast into a CNF spec manager
  CnfManager &tmp = static_cast<CnfManager &>(om);

  // allocate memory.
  unsigned pos = 0;
  m_data = new char[component.size() * sizeof(HyperEdge) +
                    sizeof(unsigned) * tmp.getSumSizeClauses()];

  // create the graph.
  for (auto &v : component) {
    // collect the edge.
    unsigned edgeData[tmp.getNbClause(v)];
    unsigned size = 0;

    for (auto &l : {Lit::makeLitFalse(v), Lit::makeLitTrue(v)}) {
      IteratorIdxClause listIdx = tmp.getVecIdxClause(l);
      for (int *ptr = listIdx.start; ptr != listIdx.end; ptr++)
        edgeData[size++] = *ptr;
    }

    // add the hyperedge.
    hypergraph.addEdge(new (&m_data[pos])
                           HyperEdge((unsigned)v, size, edgeData));
    pos += sizeof(HyperEdge) + size * sizeof(unsigned);
  }

  return {dynamic_cast<CnfManager &>(om).getNbVariable(),
          dynamic_cast<CnfManager &>(om).getNbClause(),
          dynamic_cast<CnfManager &>(om).getSumSizeClauses()};
}  // constructHyperGraph

/**
 * @brief HyperGraphExtractorCnfDual::split implementation.
 */
void HyperGraphExtractorCnfDual::split(HyperGraph &graph,
                                       std::vector<int> &partition,
                                       std::vector<Var> &cut,
                                       HyperGraph &firstGraph,
                                       HyperGraph &secondGraph) {
  if (graph.getNbEdges() < 10) {
    for (unsigned i = 0; i < graph.getNbEdges(); i++)
      cut.push_back(graph[i].getId());
    return;
  }

  for (unsigned i = 0; i < graph.getNbEdges(); i++) {
    HyperEdge &e = graph[i];
    if (!e.getSize()) continue;

    int part = partition[e[0]];
    bool clash = false;
    for (unsigned j = 1; !clash && j < e.getSize(); j++)
      clash = part != partition[e[j]];

    if (clash)
      cut.push_back(e.getId());
    else if (part == 0)
      firstGraph.addEdge(graph.getEdge(i));
    else
      secondGraph.addEdge(graph.getEdge(i));
  }

  if (!firstGraph.getNbEdges()) {
    secondGraph.setNbEdges(0);
    for (unsigned i = 0; i < graph.getNbEdges(); i++)
      cut.push_back(graph[i].getId());
  }

  if (!secondGraph.getNbEdges()) {
    firstGraph.setNbEdges(0);
    for (unsigned i = 0; i < graph.getNbEdges(); i++)
      cut.push_back(graph[i].getId());
  }
}  // split

}  // namespace d4
