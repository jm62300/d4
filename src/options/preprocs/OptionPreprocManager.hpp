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

#include "src/configurations/ConfigurationPreproc.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/preprocs/PreprocManager.hpp"

namespace d4 {

class OptionPreprocManager {
 public:
  ProblemInputType inputType;
  PreprocMethod preprocMethod;
  unsigned nbIteration = 1;
  bool onlyUseGates = false;
  bool ordered = false;
  bool strongElim = false;
  int timeout = 0;

  OptionPreprocManager(const ConfigurationPeproc& config) {
    inputType = config.inputType;
    preprocMethod = config.preprocMethod;
    nbIteration = config.nbIteration;
    onlyUseGates = config.onlyUseGates;
    ordered = config.ordered;
    timeout = config.timeout;
    strongElim = config.strongElim;
  }

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionPreprocManager& dt) {
    out << " Option PreprocManager:" << " preproc("
        << PreprocMethodManager::getPreprocMethod(dt.preprocMethod) << ')'
        << " input type(" << ProblemInputTypeManager::getInputType(dt.inputType)
        << ')';

    if (dt.preprocMethod == SHARP_EQUIV || dt.preprocMethod == COMPILE_EQUIV)
      out << " onlyGate(" << dt.onlyUseGates << ") ordered(" << dt.ordered
          << ") strong-elim(" << dt.strongElim;

    if (dt.preprocMethod != BASIC) {
      out << ") #iteration(" << dt.nbIteration << ')' << " timeout("
          << dt.timeout << ')';
    }
    return out;
  }  // <<
};
}  // namespace d4