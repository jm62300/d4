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
#include "src/bipartition/heuristic/HeuristicBipartition.hpp"

#include "src/bipartition/heuristic/HeuristicBipartitionAscOcc.hpp"
#include "src/bipartition/heuristic/HeuristicBipartitionGenTaut.hpp"
#include "src/bipartition/heuristic/HeuristicBipartitionNaturalOrder.hpp"
#include "src/bipartition/heuristic/HeuristicBipartitionRandom.hpp"
#include "src/utils/FactoryException.hpp"

namespace bipe {
namespace bipartition {

/**
 * @brief HeuristicBipartition::makeHeuristicBipartition implementation.
 */
HeuristicBipartition* HeuristicBipartition::makeHeuristicBipartition(
    Problem& p, const std::vector<Lit>& selectors, const std::string& method,
    std::ostream& out) {
  out << "c [HEURISTIC-BIPARTITION] Constructor\n";
  out << "c [HEURISTIC-BIPARTITION] Method run: " << method << "\n";

  if (method == "NATURAL_ORDER")
    return new HeuristicBipartitionNaturalOrder(p, selectors);
  else if (method == "OCC_ASC")
    return new HeuristicBipartitionAscOcc(p, selectors);
  else if (method == "RANDOM")
    return new HeuristicBipartitionRandom(p, selectors);
  else if (method == "GEN_TAUTS") {
    return new HeuristicBipartitionGenTaut(p, selectors);
  } else {
    std::string msg = "The sorting method " + method + " is unknown";
    throw(FactoryException(msg.c_str(), __FILE__, __LINE__));
  }
}  // makeHeuristicBipartition
}  // namespace bipartition
}  // namespace bipe