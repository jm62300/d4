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

#include "src/options/Option.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/options/solvers/OptionSolver.hpp"

namespace d4 {
class OptionEREMethod : public OptionRoot {
 public:
  OptionEREMethod(const std::string& name = "", const std::string& description = "ERE options")
      : OptionRoot() {
    m_name = name;
    m_description = description;
  }

  /** @brief Search for a first interpretation greedily. */
  Option<bool> greedyInitActivated{"greedyInitActivated", "Search for a first interpretation greedily", false};
  /** @brief When a decomposable AND node occurs we search for an instantiation to be able to get a bound. */
  Option<bool> digOnAnd{"digOnAnd", "Search for an instantiation to get a bound", true};
  /** @brief Search if we can find an assignment such that the number of weighted models is greater than a given threshold. */
  Option<double> threshold{"threshold", "Weighted models threshold", 1.0};
  OptionSolver optionSolver{"solver", "Solver options"};
  OptionSpecManager optionSpecManager{"spec", "Formula manager options"};

  /** @brief Activate the cutting process on the max variables regarding an upper bound. */
  Option<bool> cutExist{"cutExist", "Activate the cutting process on the exist variables", true};
  /** @brief Try to look if the best solution found so far is a good phase heuristic. */
  Option<bool> phaseHeuristicBestExist{"phaseHeuristicBestExist", "Best solution phase heuristic for exist variables", true};
  /** @brief That is the percentage of random choice for the phase selection on the exist variables. */
  Option<unsigned> randomPhaseHeuristicExist{"randomPhaseHeuristicExist", "Random phase selection for exist variables", 6};
  OptionBranchingHeuristic optionBranchingHeuristicExist{"branchingExist", "Branching options for exist variables"};
  OptionCacheManager optionCacheManagerExist{"cacheExist", "Cache options for exist variables"};

  /** @brief Compute the connected component focusing or not the set of random variables. */
  Option<bool> computeComponentOnRandom{"computeComponentOnRandom", "Compute the connected component on random variables", true};
  OptionBranchingHeuristic optionBranchingHeuristicRandom{"branchingRandom", "Branching options for random variables"};
  OptionCacheManager optionCacheManagerRandom{"cacheRandom", "Cache options for random variables"};

  std::vector<OptionBase*> getAllOptions() override {
    auto options = OptionRoot::getAllOptions();
    options.push_back((OptionBase*)&greedyInitActivated);
    options.push_back((OptionBase*)&digOnAnd);
    options.push_back((OptionBase*)&threshold);
    options.push_back((OptionBase*)&optionSolver);
    options.push_back((OptionBase*)&optionSpecManager);
    options.push_back((OptionBase*)&cutExist);
    options.push_back((OptionBase*)&phaseHeuristicBestExist);
    options.push_back((OptionBase*)&randomPhaseHeuristicExist);
    options.push_back((OptionBase*)&optionBranchingHeuristicExist);
    options.push_back((OptionBase*)&optionCacheManagerExist);
    options.push_back((OptionBase*)&computeComponentOnRandom);
    options.push_back((OptionBase*)&optionBranchingHeuristicRandom);
    options.push_back((OptionBase*)&optionCacheManagerRandom);
    return options;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionEREMethod& dt) {
    out << " Option ERE Method:";
    return out;
  }  // <<
};
}  // namespace d4