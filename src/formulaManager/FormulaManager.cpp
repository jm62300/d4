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

#include "cnf/CnfManagerDyn.hpp"
#include "cnf/CnfManagerDynBlockedCl.hpp"
#include "cnf/CnfManagerDynPure.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {

/**
 * @brief FormulaManager::makeFormulaManager implementation.
 */
FormulaManager *FormulaManager::makeFormulaManager(
    const OptionSpecManager &options, ProblemManager &p, std::ostream &out) {
  out << "c [SPEC MANAGER]" << options << "\n";

  switch (p.getProblemType()) {
    case PB_CIRC:
      out << "c TODO\n";
      exit(3);
      break;

    case PB_TCNF:
    case PB_CNF:
    case PB_QBF:
      switch (options.specUpdateType) {
        case SPEC_DYNAMIC:
          return new CnfManagerDyn(p);
        case SPEC_DYNAMIC_BLOCKED_SIMP:
          return new CnfManagerDynBlockedCl(p);
        case SPEC_DYNAMIC_PURE_SIMP:
          return new CnfManagerDynPure(p);
      }
    case PB_NONE:
      std::cerr << "c The problem type has to be specified\n";
      ProblemInputTypeManager::displayPossibleOptions(std::cerr);
      exit(ERROR_BAD_OPTION);
  }

  throw(FactoryException("Cannot create a FormulaManager", __FILE__, __LINE__));
}  // makeFormulaManager

/**
 * @brief FormulaManager::showTrail implementation.
 */
void FormulaManager::showTrail(std::ostream &out) {
  for (int i = 0; i < getNbVariable(); i++) {
    if (!varIsAssigned(i)) continue;
    Lit l = Lit::makeLit(i, false);
    if (litIsAssignedToTrue(l))
      out << l << " ";
    else
      out << ~l << " ";
  }
  out << "\n";
}  // showFormula

}  // namespace d4
