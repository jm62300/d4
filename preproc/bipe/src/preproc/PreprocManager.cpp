/**
 * eliminator
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

#include "PreprocManager.hpp"

#include "src/reducer/Method.hpp"
#include "src/utils/Problem.hpp"

namespace bipe {

/**
 * @brief Executes a lightweight preprocessing phase on the given problem.
 *
 * This method is considered "light" because it avoids the heavy computational
 * overhead of invoking a full SAT solver. Instead, it relies on a dedicated
 * clause reducer (using a combination of fast reduction techniques like
 * vivification or occurrence elimination) to simplify the formula in-place.
 *
 * @param[in,out] problem The SAT problem containing the clauses to be
 * simplified. The simplified clauses will replace the original ones.
 * @param[in] optionPreproc Configuration options controlling limits, timeouts,
 *                          and verbosity for the preprocessing phase.
 */
void preprocLight(Problem& problem, const OptionPreproc& optionPreproc) {
  // Initialize and start the execution timer based on user options
  if (optionPreproc.verbose) std::cout << "c [PREPROC] Preproc Light\n";

  Timer timer(optionPreproc.timeout);
  if (optionPreproc.timeout != 0) timer.start();

  // Instantiate the reducer method using a factory
  bipe::reducer::Method* rm = bipe::reducer::Method::makeMethod(
      optionPreproc.optionReducer.reducerName, std::cout);

  // Execute the reducer on the problem's clauses.
  rm->run(problem.getNbVar(), problem.getClauses(),
          optionPreproc.optionReducer.nbIterarion, optionPreproc.verbose,
          problem.getClauses(), timer);

  delete rm;
}  // preprocLight

/**
 * @brief PreprocManager::run implementation.
 */
void PreprocManager::run(unsigned nbVar, std::vector<std::vector<int>>& clauses,
                         std::vector<int> projected, std::vector<int> protect,
                         const PreprocMethod& preprocMethod,
                         const OptionPreproc& optionPreproc) {
  Problem problem(nbVar, clauses, projected, protect);

  switch (preprocMethod) {
    case EQUIV_LIGHT:
      preprocLight(problem, optionPreproc);
      break;
    case NONE:
      if (optionPreproc.verbose) std::cout << "c [PREPROC] None\n";
      return;
  }

  // get the new clauses.
  clauses.clear();
  clauses.reserve(problem.getClauses().size());
  for (auto& cl : problem.getClauses()) {
    std::vector<int> newcl;
    for (auto& l : cl) newcl.push_back(l.human());
    clauses.push_back(newcl);
  }
}  // run
}  // namespace bipe