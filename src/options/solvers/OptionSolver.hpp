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

#include <string>

#include "src/exceptions/FactoryException.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/OptionRegistry.hpp"
#include "src/options/EnumMetadata.hpp"

namespace d4 {
enum SolverName { GLUCOSE_CNF, MINISAT_CNF };

template <>
struct EnumMetadata<SolverName> {
  static std::string name() { return "SolverName"; }
  static std::map<int, std::string> mapping() {
    return {{GLUCOSE_CNF, "glucose"}, {MINISAT_CNF, "minisat"}};
  }
};

class OptionSolver : public OptionGroup {
 public:
  OptionSolver(const std::string& name = "solver", const std::string& description = "Solver options")
      : OptionGroup(name, description) {}

  /** @brief The solver we will use */
  Option<SolverName> solverName{"solverName", "The solver we will use", MINISAT_CNF};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&solverName};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionSolver& dt) {
    out << " Option Solver:"
        << " solver name(" << dt.solverName.getValueAsString()
        << ")";

    return out;
  }  // <<
};

}  // namespace d4