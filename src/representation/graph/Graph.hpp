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

#include "vector"

namespace d4 {

class Graph {
 private:
  unsigned m_nbNode;
  std::vector<std::pair<unsigned, unsigned>> m_edges;

 public:
  /**
   * @brief Construct a new Graph object.
   *
   * @param nbNode is the number of nodes in the graph.
   */
  Graph(unsigned nbNode) : m_nbNode(nbNode) {}

  /**
   * @brief Add an edge into the graph.
   *
   * @param edge is the edge we want to add.
   */
  void addEdge(const std::pair<unsigned, unsigned> &edge);

  /**
   * @brief Get the number of nodes.
   *
   * @return the number of nodes.
   */
  inline unsigned getNbNode() { return m_nbNode; }

  /**
   * @brief Get the edge list.
   *
   * @return the edges.
   */
  inline std::vector<std::pair<unsigned, unsigned>> &getEdge() {
    return m_edges;
  }  // getEdge
};
}  // namespace d4