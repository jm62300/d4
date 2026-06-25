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

#include <iostream>
#include <string>

#include "src/options/OptionRoot.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/solvers/OptionSolver.hpp"

namespace d4 {
class OptionQbfCounter : public OptionRoot {
 public:
  OptionQbfCounter(const std::string& name = "", const std::string& description = "QBF counter options")
      : OptionRoot() {
    m_name = name;
    m_description = description;
  }

  OptionCacheManager optionCacheManager{"cache", "Cache options"};
  OptionSolver optionSolver{"solver", "Solver options"};
  OptionSpecManager optionSpecManager{"spec", "Formula manager options"};
  OptionBranchingHeuristic optionBranchingHeuristic{"branching", "Branching options"};
  OptionPartialOrderHeuristic optionPartitioningHeuristic{"partitioning", "Partitioning options"};

  std::vector<OptionBase*> getAllOptions() override {
    auto options = OptionRoot::getAllOptions();
    options.push_back((OptionBase*)&optionCacheManager);
    options.push_back((OptionBase*)&optionSolver);
    options.push_back((OptionBase*)&optionSpecManager);
    options.push_back((OptionBase*)&optionBranchingHeuristic);
    options.push_back((OptionBase*)&optionPartitioningHeuristic);
    return options;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionQbfCounter& dt) {
    out << " Option QBF Counter: ";
    return out;
  }  // <<
};

}  // namespace d4