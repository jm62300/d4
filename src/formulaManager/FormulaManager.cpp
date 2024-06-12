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
#include "FormulaManager.hpp"

#include "cnf/SpecManagerCnfDyn.hpp"
#include "cnf/SpecManagerCnfDynBlockedCl.hpp"
#include "cnf/SpecManagerCnfDynPure.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

/**
 * @brief FormulaManager::makeSpecManager implementation.
 */
FormulaManager *FormulaManager::makeSpecManager(
    const OptionSpecManager &options, ProblemManager &p, std::ostream &out) {
  out << "c [SPEC MANAGER]" << options << "\n";

  if (p.getProblemType() == PB_CNF || p.getProblemType() == PB_QBF ||
      p.getProblemType() == PB_CIRC) {
    if (p.getProblemType() == PB_CIRC)
      out << "c Warning: only handle the case where the circuit is translated "
             "into a CNF formula\n";
    if (options.specUpdateType == SPEC_DYNAMIC) return new SpecManagerCnfDyn(p);
    if (options.specUpdateType == SPEC_DYNAMIC_BLOCKED_SIMP)
      return new SpecManagerCnfDynBlockedCl(p);
    if (options.specUpdateType == SPEC_DYNAMIC_PURE_SIMP)
      return new SpecManagerCnfDynPure(p);
  }

  throw(FactoryException("Cannot create a FormulaManager", __FILE__, __LINE__));
}  // makeSpecManager

}  // namespace d4
