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
HyperGraph::HyperGraph()
    : m_edges(NULL),
      m_nbEdges(0),
      m_capacityEdge(0),
      m_memory(NULL),
      m_sizeMemory(0),
      m_capacityMemory(0) {}

/**
   Free the allocated memory.
 */
HyperGraph::~HyperGraph() {
  if (m_edges) delete[] m_edges;
  if (m_memory) delete[] m_memory;
}  // destructor

/**
 * @brief HyperGraph::display implementation.
 */
void HyperGraph::display(std::ostream &out) {
  for (unsigned i = 0; i < m_nbEdges; i++) {
    for (unsigned j = 0; j < m_edges[i]->getSize(); j++)
      out << (*m_edges[i])[j] << " ";
    out << "\n";
  }
}  // displayHyperGraph

/**
 * @brief HyperGraph::addEdge implementation.
 */
void HyperGraph::addEdge(const HyperEdge &e) {
  assert(m_nbEdges <= m_capacityEdge);
  if (m_nbEdges == m_capacityEdge) {
    m_capacityMemory += s_BLOC_SIZE_EDGE;
    m_edges =
        (HyperEdge **)realloc(m_edges, sizeof(HyperEdge *) * m_capacityEdge);
  }

  if (m_sizeMemory + e.getSize() > m_capacityMemory) {
    m_capacityMemory += e.getSize() / s_BLOC_MEMORY + s_BLOC_MEMORY;
    m_memory = (char *)realloc(m_memory, sizeof(m_capacityMemory));
  }

  m_sumEdgeSize += e.getSize();
  m_edges[m_nbEdges++] = new (&m_memory[m_sizeMemory]) HyperEdge(e);
  m_sizeMemory += sizeof(HyperEdge) + sizeof(unsigned) * e.getSize();
}  // addEdge

/**
 * @brief HyperGraph::addEdge implementation.
 */
void HyperGraph::addEdge(HyperEdge *e) {
  assert(m_nbEdges <= m_capacityEdge);
  if (m_nbEdges == m_capacityEdge) {
    m_capacityMemory += s_BLOC_SIZE_EDGE;
    m_edges =
        (HyperEdge **)realloc(m_edges, sizeof(HyperEdge *) * m_capacityEdge);
  }

  m_sumEdgeSize += e->getSize();
  m_edges[m_nbEdges++] = e;
}  // addEdge

}  // namespace d4
