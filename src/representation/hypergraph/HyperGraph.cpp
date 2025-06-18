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

#include "HyperGraph.hpp"

namespace d4 {
/**
 * @brief HyperGraph::HyperGraph implementation.
 */
HyperGraph::HyperGraph() : m_edges(NULL), m_nbEdges(0), m_capacityEdge(0) {}

/**
 * @brief HyperGraph::HyperGraph implementation.
 */
HyperGraph::HyperGraph(unsigned capacity) {
  m_edges = (HyperEdge **)malloc(capacity * sizeof(HyperEdge *));
  m_nbEdges = 0;
  m_capacityEdge = capacity;
}  // constructor

/**
 * @brief HyperGraph::HyperGraph implementation.
 */
HyperGraph::HyperGraph(const HyperGraph &g) : HyperGraph(g.getNbEdges()) {
  for (unsigned i = 0; i < g.getNbEdges(); i++) addEdge(g.getEdge(i));
  assert(g.getNbEdges() == getNbEdges());
}  // constructor

/**
   Free the allocated memory.
 */
HyperGraph::~HyperGraph() {
  if (m_edges) free(m_edges);
}  // destructor

/**
 * @brief HyperGraph::display implementation.
 */
void HyperGraph::display(std::ostream &out) {
  for (unsigned i = 0; i < m_nbEdges; i++) {
    HyperEdge &e = *m_edges[i];
    for (unsigned j = 0; j < e.getSize(); j++) out << e[j] << " ";
    out << "\n";
  }
}  // displayHyperGraph

/**
 * @brief HyperGraph::addEdge implementation.
 */
void HyperGraph::addEdge(HyperEdge *e) {
  assert(m_nbEdges <= m_capacityEdge);
  if (m_nbEdges >= m_capacityEdge) {
    m_capacityEdge += s_BLOC_SIZE_EDGE;
    m_edges =
        (HyperEdge **)realloc(m_edges, sizeof(HyperEdge *) * m_capacityEdge);
  }

  m_sumEdgeSize += e->getSize();
  m_edges[m_nbEdges++] = e;
}  // addEdge

}  // namespace d4
