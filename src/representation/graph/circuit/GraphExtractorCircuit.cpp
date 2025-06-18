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

#include "GraphExtractorCircuit.hpp"

namespace d4 {

/**
 * @brief GraphExtractorCnf::extractCnf implementation.
 */
void GraphExtractorCircuit::extractCircuit(
    CircuitManager &formula, std::vector<Var> &component,
    std::vector<std::vector<int> > &gates) {
  for (auto &g : formula.getGates()) {
    if (!formula.isActiveGates(g)) continue;
    std::vector<int> gate;
    gate.push_back(g.output.var());
    for (auto &l : g.input) gate.push_back(l.var());
    gates.push_back(gate);
  }
}  // extractCircuit

}  // namespace d4