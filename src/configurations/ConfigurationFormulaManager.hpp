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

#include "src/options/formulaManager/OptionFormulaManager.hpp"

namespace d4 {
struct ConfigurationSpec {
  /** @brief The occurrence manager used (dynamic, dynamicBlockedSimp or dynamicPureSimp).  */
  SpecUpdateType specUpdateType = SPEC_DYNAMIC;
  /** @brief If this option is activated and if the problem is a circuit, then some gates can be removed during the search if those ones are not active. */
  bool removeGates = false;
  bool needFastNotSatisfied = false;
};
}  // namespace d4