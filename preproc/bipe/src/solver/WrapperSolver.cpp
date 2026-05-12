/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "src/solver/WrapperSolver.hpp"

#include "src/solver/WrapperGlucose.hpp"
#include "src/utils/FactoryException.hpp"

namespace bipe {
/**
 * @brief WrapperSolver::makeWrapperSolver implementation.
 */
WrapperSolver* WrapperSolver::makeWrapperSolver(const std::string solverName,
                                                std::ostream& out) {
  out << "c [CONSTRUCTOR] Solver: " << solverName << "\n";

  if (solverName == "glucose") return new WrapperGlucose();

  throw(FactoryException("Cannot create a WrapperSolver", __FILE__, __LINE__));
}  // makeWrapperSolver

}  // namespace bipe
