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
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/options/formulaManager/OptionFormulaManager.hpp"
#include "src/solvers/OptionSolver.hpp"

namespace d4 {
class OptionProjMcMethod : public OptionRoot {
 public:
  OptionProjMcMethod(const std::string& name = "", const std::string& description = "ProjMC options")
      : OptionRoot() {
    m_name = name;
    m_description = description;
  }

  Option<bool> refinement{"refinement", "Refinement activated or not", true};
  OptionCacheManager optionCache{"cache", "Cache options"};
  OptionSolver optionSolver{"solver", "Solver options"};
  OptionSpecManager optionSpecs{"spec", "Formula manager options"};
  OptionDpllStyleMethod optionCounter{"counter", "Counter options"};

  std::vector<OptionBase*> getAllOptions() override {
    auto options = OptionRoot::getAllOptions();
    options.push_back((OptionBase*)&refinement);
    options.push_back((OptionBase*)&optionCache);
    options.push_back((OptionBase*)&optionSolver);
    options.push_back((OptionBase*)&optionSpecs);
    options.push_back((OptionBase*)&optionCounter);
    return options;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionProjMcMethod& dt) {
    out << " Option ProjMC Method:"
        << " refinement(" << dt.refinement.get() << ")"
        << " cache(" << dt.optionCache << ")"
        << " solver(" << dt.optionSolver << ")"
        << " spec(" << dt.optionSpecs << ")"
        << " counter(" << dt.optionCounter << ")";
    return out;
  }  // <<
};
}  // namespace d4