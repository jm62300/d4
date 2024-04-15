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

#include "HyperEdge.hpp"

#include <iostream>

namespace d4 {

/**
 * @brief HyperEdge::HyperEdge implementation.
 */
HyperEdge::HyperEdge(const HyperEdge &e) {
  m_id = e.getId();
  m_size = e.getSize();

  // we suppose the memory has been allocated somewhere.
  for (unsigned i = 0; i < e.getSize(); i++) m_data[i] = e[i];
}  // constructor

/**
 * @brief HyperEdge::HyperEdge implementation.
 */
HyperEdge::HyperEdge(unsigned id, unsigned size, unsigned *data)
    : m_id(id), m_size(size) {
  for (unsigned i = 0; i < size; i++) m_data[i] = data[i];
}  // constructor

}  // namespace d4
