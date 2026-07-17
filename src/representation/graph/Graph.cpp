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

#include "Graph.hpp"

namespace d4 {
/**
 * @brief Graph::addEdge implementation.
 */
void Graph::addEdge(const std::pair<unsigned, unsigned> &edge) {
  unsigned l = edge.first, r = edge.second;
  if (l > r) std::swap(l, r);

  if (m_adjList[l].find(r) == m_adjList[l].end()) {
    m_edges.push_back(std::make_pair(l, r));
    m_adjList[l].insert(r);
  }
}  // addEdge

/**
 * @brief Graph::display implementation.
 */
void Graph::display(std::ostream &out) {
  out << "#nodes: " << m_nbNode << "\n";
  for (auto &p : m_edges) out << p.first << " " << p.second << '\n';
}  // display

}  // namespace d4