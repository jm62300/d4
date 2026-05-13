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
class OptionMinSharpSatMethod : public OptionRoot {
 public:
  OptionMinSharpSatMethod(const std::string& name = "", const std::string& description = "MinSharpSAT options")
      : OptionRoot() {
    m_name = name;
    m_description = description;
  }

  Option<bool> greedyInitActivated{"greedyInitActivated", "Search for a first interpretation greedily", false};
  Option<bool> digOnAnd{"digOnAnd", "Search for an instantiation to get a bound", true};
  Option<double> threshold{"threshold", "Weighted models threshold", 1.0};
  OptionSolver optionSolver{"solver", "Solver options"};
  OptionSpecManager optionSpecManager{"spec", "Formula manager options"};

  Option<std::string> phaseHeuristicMin{"phaseHeuristicMin", "The heuristic used to select the phase of the MIN variables", "best"};
  Option<unsigned> randomPhaseHeuristicMin{"randomPhaseHeuristicMin", "Random phase heuristic percentage for MIN variables", 6};
  OptionBranchingHeuristic optionBranchingHeuristicMin{"branchingMin", "Branching options for MIN variables"};
  OptionCacheManager optionCacheManagerMin{"cacheMin", "Cache options for MIN variables"};

  OptionBranchingHeuristic optionBranchingHeuristicInd{"branchingInd", "Branching options for IND variables"};
  OptionCacheManager optionCacheManagerInd{"cacheInd", "Cache options for IND variables"};

  std::vector<OptionBase*> getAllOptions() override {
    auto options = OptionRoot::getAllOptions();
    options.push_back((OptionBase*)&greedyInitActivated);
    options.push_back((OptionBase*)&digOnAnd);
    options.push_back((OptionBase*)&threshold);
    options.push_back((OptionBase*)&optionSolver);
    options.push_back((OptionBase*)&optionSpecManager);
    options.push_back((OptionBase*)&phaseHeuristicMin);
    options.push_back((OptionBase*)&randomPhaseHeuristicMin);
    options.push_back((OptionBase*)&optionBranchingHeuristicMin);
    options.push_back((OptionBase*)&optionCacheManagerMin);
    options.push_back((OptionBase*)&optionBranchingHeuristicInd);
    options.push_back((OptionBase*)&optionCacheManagerInd);
    return options;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionMinSharpSatMethod& dt) {
    out << " Option MinSharpSAT Method:";
    return out;
  }  // <<
};
}  // namespace d4
