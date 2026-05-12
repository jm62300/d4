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

#include <cstring>
#include <vector>

#include "src/utils/Problem.hpp"
namespace bipe {

/**
 * @brief Handles the generation and extraction of structural symmetries from a
 * CNF formula.
 *
 * This class interfaces with an external symmetry-detection tool by invoking it
 * via the filesystem. It reads the resulting symmetry groups, which can then be
 * used to dramatically reduce the search space in subsequent SAT solving or
 * bipartitioning steps.
 */
class SymGenerate {
 public:
  /**
   * @brief Retrieves the symmetry groups contained within the specified
   * problem.
   *
   * @param path The file path to the external executable program used to
   * compute the symmetries.
   * @param file The file path to the input CNF problem (must be in standard
   * DIMACS format).
   * @param verb If true, outputs detailed progress logs and information during
   * the execution.
   * @param nbVar The total number of variables present in the given CNF
   * formula.
   * @param[out] symGroup A nested vector where the extracted symmetry groups
   * (generators) will be populated and stored.
   */
  void getSymmetries(const std::string& path, const std::string& file,
                     bool verb, unsigned nbVar,
                     std::vector<std::vector<Var>>& symGroup);
};

}  // namespace bipe