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
#include "src/options/EnumMetadata.hpp"
#include "src/options/Option.hpp"
#include "src/options/OptionGroup.hpp"
#include "src/options/OptionRegistry.hpp"

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
  OptionSolver(const std::string& name = "solver",
               const std::string& description = "Solver options")
      : OptionGroup(name, description) {}

  Option<SolverName> solverName{
      "solverName", "The solver we will use", GLUCOSE_CNF};

  Option<unsigned> initBudget{
      "solver-init-budget",
      "Conflict budget for the primary solver before CaDiCaL fallback", 500};

  Option<unsigned> minLimitVar{
      "solver-min-limit-var",
      "Minimum component size before applying conflict budget", 50};

  Option<unsigned> learntFactor{
      "solver-learnt-factor",
      "Remove learnt clauses when #learnt > factor * #initial-clauses", 1};

  Option<unsigned> cadicalRedundantFactor{
      "solver-cadical-redundant-factor",
      "Rebuild CaDiCaL when #redundant > factor * #initial-clauses", 1};

  std::vector<OptionBase*> getAllOptions() override {
    return {(OptionBase*)&solverName,    (OptionBase*)&initBudget,
            (OptionBase*)&minLimitVar,   (OptionBase*)&learntFactor,
            (OptionBase*)&cadicalRedundantFactor};
  }

  friend std::ostream& operator<<(std::ostream& out, const OptionSolver& dt) {
    out << " Option Solver:"
        << " solver(" << dt.solverName.getValueAsString() << ")"
        << " init-budget(" << (unsigned)dt.initBudget << ")"
        << " min-limit-var(" << (unsigned)dt.minLimitVar << ")"
        << " learnt-factor(" << (unsigned)dt.learntFactor << ")"
        << " cadical-redundant-factor(" << (unsigned)dt.cadicalRedundantFactor
        << ")";
    return out;
  }
};

}  // namespace d4
