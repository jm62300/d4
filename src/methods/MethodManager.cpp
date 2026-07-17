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

#include "MethodManager.hpp"

#include "DpllStyleMethod.hpp"
// #include "ExistRandomExist.hpp"
// #include "MaxSharpSAT.hpp"
// #include "MinSharpSAT.hpp"
// #include "OperationManager.hpp"
#include "src/exceptions/BadBehaviourException.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/options/methods/OptionMethodManager.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {

/**
 * @brief Display the projected variables in order.
 * @param[in] selected The list of projected variables.
 * @param[in] out The stream where is printed out the information.
 */
void MethodManager::displayInfoVariables(const ProblemManager& problem,
                                         std::ostream& out) {
  std::cout << "c Display the quantification problem:\n";
  unsigned i = 0;
  for (auto& level : problem.getQuantification()) {
    std::cout << "c level " << i++ << '\n';
    for (auto& v : level) std::cout << v << ' ';
    std::cout << '\n';
  }
}  // displayInfoProjected

}  // namespace d4
