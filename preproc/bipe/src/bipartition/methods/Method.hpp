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
#pragma once

#include <chrono>

#include "src/bipartition/option/Option.hpp"
#include "src/bipartition/option/OptionBackbone.hpp"
#include "src/bipartition/option/OptionBipartition.hpp"
#include "src/bipartition/option/OptionDac.hpp"
#include "src/utils/Gate.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/Timer.hpp"

namespace bipe {

class WrapperSolver;

namespace bipartition {
class Backbone;
class DACircuit;

/**
 * @brief Abstract base class for bipartition and preprocessing methods.
 *
 * Provides common utilities and shared simplification routines (like
 * bounded SAT solver calls, backbone extraction, and DAC extraction)
 * used by various bipartition algorithms.
 */
class Method {
 protected:
  unsigned m_timeLimit = 0;
  std::chrono::time_point<std::chrono::system_clock> m_start, m_end;

  class WrapperSolver* m_solver = nullptr;
  class Backbone* m_backboneMethod = nullptr;
  class DACircuit* m_dacMethod = nullptr;

 public:
  /**
   * @brief Virtual destructor to ensure proper cleanup of derived classes.
   */
  virtual ~Method();

  /**
   * @brief Collects input variables from detected unit literals.
   *
   * This is used as a fallback mechanism to safely extract the known inputs
   * in scenarios where the preprocessor detects units but receives a signal
   * to halt its execution prematurely.
   *
   * @param p The problem instance currently being analyzed for a bipartition.
   * @param units The set of unit literals found so far.
   * @param[out] input The set of input variables to be populated.
   */
  void constructInputFromUnits(Problem& p, std::vector<Lit>& units,
                               std::vector<Var>& input);

  /**
   * @brief Executes a single, optionally bounded, SAT solver call to simplify
   * the formula.
   *
   * @param p The problem instance to simplify.
   * @param solverName The string identifier of the SAT solver backend to
   * instantiate.
   * @param nbConflict The maximum number of conflicts the solver is allowed to
   * execute.
   * @param[out] units The extracted unit clauses, formatted as gates.
   * @param out The output stream used for logging progress and statistics.
   * @param[out] setOfModels A collection used to store any valid models found
   * during the solving process.
   * @param timer The timer used to strictly enforce execution time limits.
   * @return A dynamically allocated, simplified version of the Problem if it is
   * SAT, or nullptr if the problem is proven UNSAT.
   */
  Problem* simplifyOneCall(Problem& p, const std::string& solverName,
                           const unsigned nbConflict, std::vector<Gate>& units,
                           std::ostream& out,
                           std::vector<std::vector<bool>>& setOfModels,
                           const Timer& timer);

  /**
   * @brief Simplifies the formula by extracting the backbone of the problem.
   *
   * Calls the backbone extractor method (which can be resource-limited) to
   * deduce forced variable assignments and reduce the problem size.
   *
   * @param p The problem instance to simplify.
   * @param optBackbone Configuration options and parameters for the backbone
   * extraction method.
   * @param[out] units The extracted backbone variables, formatted as gates.
   * @param out The output stream used for logging progress and statistics.
   * @param[out] setOfModels A collection used to store any valid models found
   * during the solving process.
   * @param timer The timer used to strictly enforce execution time limits.
   * @return A dynamically allocated, simplified version of the Problem if it is
   * SAT, or nullptr if the problem is proven UNSAT.
   */
  Problem* simplifyBackbone(Problem& p, const OptionBackbone& optBackbone,
                            std::vector<Gate>& units, std::ostream& out,
                            std::vector<std::vector<bool>>& setOfModels,
                            const Timer& timer);

  /**
   * @brief Simplifies the formula by extracting a Directed Acyclic Circuit
   * (DAC).
   *
   * Identifies functional dependencies (logic gates) within the CNF and uses
   * them to eliminate variables and simplify the problem structure.
   *
   * @param p The problem instance to simplify.
   * @param optionDac Configuration options and parameters for the DAC
   * extraction method.
   * @param[out] units The extracted gates and functional dependencies.
   * @param out The output stream used for logging progress and statistics.
   * @param[out] setOfModels A collection used to store any valid models found
   * during the solving process.
   * @param timer The timer used to strictly enforce execution time limits.
   * @return A dynamically allocated, simplified version of the Problem if it is
   * SAT, or nullptr if the problem is proven UNSAT.
   */
  Problem* simplifyDac(Problem& p, const OptionDac& optionDac,
                       std::vector<Gate>& units, std::ostream& out,
                       std::vector<std::vector<bool>>& setOfModels,
                       const Timer& timer);
};

}  // namespace bipartition
}  // namespace bipe