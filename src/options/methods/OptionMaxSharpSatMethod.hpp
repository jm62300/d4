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

#include "src/options/OptionRoot.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/solvers/OptionSolver.hpp"

namespace d4 {
class OptionMaxSharpSatMethod : public OptionRoot {
 public:
  OptionMaxSharpSatMethod(const std::string& name = "", const std::string& description = "MaxSharpSAT options")
      : OptionRoot() {
    m_name = name;
    m_description = description;
  }

  Option<bool> greedyInitActivated{"greedyInitActivated", "Search for a first interpretation greedily", false};
  Option<bool> digOnAnd{"digOnAnd", "Search for an instantiation to get a bound", true};
  Option<double> threshold{"threshold", "Weighted models threshold", 1.0};
  OptionSolver optionSolver{"solver", "Solver options"};
  OptionSpecManager optionSpecManager{"spec", "Formula manager options"};

  Option<std::string> phaseHeuristicMax{"phaseHeuristicMax", "The heuristic used to select the phase of the MAX variables", "best"};
  Option<unsigned> randomPhaseHeuristicMax{"randomPhaseHeuristicMax", "Random phase heuristic percentage for MAX variables", 6};
  OptionBranchingHeuristic optionBranchingHeuristicMax{"branchingMax", "Branching options for MAX variables"};
  OptionCacheManager optionCacheManagerMax{"cacheMax", "Cache options for MAX variables"};

  OptionBranchingHeuristic optionBranchingHeuristicInd{"branchingInd", "Branching options for IND variables"};
  OptionCacheManager optionCacheManagerInd{"cacheInd", "Cache options for IND variables"};

  std::vector<OptionBase*> getAllOptions() override {
    auto options = OptionRoot::getAllOptions();
    options.push_back((OptionBase*)&greedyInitActivated);
    options.push_back((OptionBase*)&digOnAnd);
    options.push_back((OptionBase*)&threshold);
    options.push_back((OptionBase*)&optionSolver);
    options.push_back((OptionBase*)&optionSpecManager);
    options.push_back((OptionBase*)&phaseHeuristicMax);
    options.push_back((OptionBase*)&randomPhaseHeuristicMax);
    options.push_back((OptionBase*)&optionBranchingHeuristicMax);
    options.push_back((OptionBase*)&optionCacheManagerMax);
    options.push_back((OptionBase*)&optionBranchingHeuristicInd);
    options.push_back((OptionBase*)&optionCacheManagerInd);
    return options;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionMaxSharpSatMethod& dt) {
    out << " Option MaxSharpSAT Method:";
    return out;
  }  // <<
};
}  // namespace d4
