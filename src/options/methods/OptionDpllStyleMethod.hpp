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

#include "src/exceptions/FactoryException.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/options/OptionRoot.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/options/methods/OptionOperationManager.hpp"
#include "src/options/solvers/OptionSolver.hpp"

namespace d4 {
class OptionDpllStyleMethod : public OptionRoot {
 public:
  OptionDpllStyleMethod(
      const std::string& name = "dpll",
      const std::string& description = "DPLL-style counter options")
      : OptionRoot() {
    m_name = name;
    m_description = description;
  }

  Option<OperationType> operationType{"operationType", "The operation type",
                                      OP_COUNTING};
  OptionCacheManager optionCacheManager{"cache", "Cache options"};
  OptionSolver optionSolver{"solver", "Solver options"};
  OptionSpecManager optionSpecManager{"spec", "Formula manager options"};
  OptionBranchingHeuristic optionBranchingHeuristic{"branching",
                                                    "Branching options"};
  Option<bool> exploitModel{"exploitModel", "If we exploit model during search",
                            true};

  std::vector<OptionBase*> getAllOptions() override {
    auto options = OptionRoot::getAllOptions();
    options.push_back((OptionBase*)&operationType);
    options.push_back((OptionBase*)&optionCacheManager);
    options.push_back((OptionBase*)&optionSolver);
    options.push_back((OptionBase*)&optionSpecManager);
    options.push_back((OptionBase*)&optionBranchingHeuristic);
    options.push_back((OptionBase*)&exploitModel);
    return options;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionDpllStyleMethod& dt) {
    out << static_cast<const OptionRoot&>(dt);
    out << " Option DPLL-style Method: exploit-model(" << dt.exploitModel.get()
        << ") verbosity(" << dt.verbosity.get() << ")\n";
    out << " Operation: "
        << OperationTypeManager::getOperatorType(dt.operationType.get())
        << "\n";
    out << dt.optionCacheManager << "\n";
    out << dt.optionSolver << "\n";
    out << dt.optionSpecManager << "\n";
    out << dt.optionBranchingHeuristic << "\n";
    return out;
  }  // <<
};

}  // namespace d4