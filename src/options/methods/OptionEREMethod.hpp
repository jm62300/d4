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

#include "src/configurations/ConfigurationEREMethod.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/options/solvers/OptionSolver.hpp"

namespace d4 {
class OptionEREMethod {
 public:
  /** @brief Search for a first interpretation greedily. */
  bool greedyInitActivated;
  /** @brief When a decomposable AND node occurs we search for an instantiation to be able to get a bound. */
  bool digOnAnd;
  /** @brief Search if we can find an assignment such that the number of weighted models is greater than a given threshold. */
  double threshold;
  OptionSolver optionSolver;
  OptionSpecManager optionSpecManager;

  /** @brief Activate the cutting process on the max variables regarding an upper bound. */
  bool cutExist;
  /** @brief Try to look if the best solution found so far is a good phase heuristic. */
  bool phaseHeuristicBestExist;
  /** @brief That is the percentage of random choice for the phase selection on the exist variables. */
  unsigned randomPhaseHeuristicExist;
  OptionBranchingHeuristic optionBranchingHeuristicExist;
  OptionCacheManager optionCacheManagerExist;

  /** @brief Compute the connected component focusing or not the set of random variables. */
  bool computeComponentOnRandom;
  OptionBranchingHeuristic optionBranchingHeuristicRandom;
  OptionCacheManager optionCacheManagerRandom;

  /**
   * @brief Construct a new object with the default parameters.
   */
  OptionEREMethod() : OptionEREMethod(ConfigurationEREMethod()) {}

  /**
   * @brief Construct an OptionEREMethd from a configuration.
   *
   * @param config gives the method configuration.
   */
  OptionEREMethod(const ConfigurationEREMethod& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionEREMethod& dt) {
    out << " Option ERE Method:";
    return out;
  }  // <<
};
}  // namespace d4