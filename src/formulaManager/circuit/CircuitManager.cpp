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

#include "CircuitManager.hpp"

namespace d4 {

/**
 * @brief CircuitManager::CircuitManager implementation.
 */
CircuitManager::CircuitManager(ProblemManager &p)
    : FormulaManager(p.getNbVar()) {
  std::cout << "c [CIRCUIT MANAGER] Constructor called\n";

  try {
    ProblemManagerCircuit &pcirc = dynamic_cast<ProblemManagerCircuit &>(p);
    m_gates = pcirc.getGates();
    m_true_lits = pcirc.getTrueLiterals();
  } catch (std::bad_cast &bc) {
    std::cerr << "c bad_cast caught: " << bc.what() << '\n';
    std::cerr << "c A boolean circuit was expected\n";
    assert(0);
  }
}  // constructor
}  // namespace d4
