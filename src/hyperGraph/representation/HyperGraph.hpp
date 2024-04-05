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

#include <cassert>
#include <iostream>
#include <iterator>

#include "HyperEdge.hpp"

namespace d4 {
class HyperGraph {
 private:
  static const unsigned s_BLOC_SIZE_EDGE = 1024;
  static const unsigned s_BLOC_MEMORY = 16384;

  HyperEdge **m_edges;
  unsigned m_nbEdges;
  unsigned m_capacityEdge;
  char *m_memory;
  unsigned m_sizeMemory;
  unsigned m_capacityMemory;

 public:
  HyperGraph();
  ~HyperGraph();

  inline unsigned getNbEdges() { return m_nbEdges; }
  inline HyperEdge **getEdges() { return m_edges; }

  inline HyperEdge &operator[](unsigned i) {
    assert(i < m_nbEdges);
    return *(m_edges[i]);
  }

  /**
   * @brief Display the hpyergraph.
   *
   * @param[in] out is the output stream used for printing out the hypergraph.
   */
  void display(std::ostream &out = std::cout);

  /**
   * @brief Add an edge to the hypergraph (the memory will be allocated for this
   * edge).
   *
   * @param e is the edge we want to add.
   */
  void addEdge(const HyperEdge &e);

  /**
   * @brief Add an edge to the hypergraph (we suppose the memory has already
   * been allocated for this edge).
   *
   * @param e is the edge we want to add.
   */
  void addEdge(HyperEdge *e);
};
}  // namespace d4
