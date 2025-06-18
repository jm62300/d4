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
#include <iterator>

namespace d4 {

class HyperEdge {
 private:
  unsigned m_id;
  unsigned m_size;
  unsigned m_data[0];

 public:
  /**
   * @brief Construct a new Hyper Edge by copy.
   *
   * @param e is the edge we want to copy.
   */
  HyperEdge(const HyperEdge &e);

  /**
   * @brief Construct a new Hyper Edge object.
   *
   * @param id is the hyperedge id.
   * @param size is the size of the edge.
   * @param data gives the elements.
   */
  HyperEdge(unsigned id, unsigned size, unsigned *data);

  inline unsigned getId() const { return m_id; }
  inline unsigned getSize() const { return m_size; }
  inline unsigned *getData() { return m_data; }

  inline unsigned operator[](unsigned i) const { return m_data[i]; }
};

}  // namespace d4
