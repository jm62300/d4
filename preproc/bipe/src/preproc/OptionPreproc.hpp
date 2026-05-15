/**
 * eliminator
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <iostream>

#include "src/bipartition/methods/Backbone.hpp"
#include "src/bipartition/methods/DACircuit.hpp"

namespace bipe {
struct OptionReducer {
  std::string reducerName = "combinaison";
  int nbIterarion = 10;
};

struct OptionPreproc {
  unsigned timeout = 0;
  bool verbose = true;
  OptionReducer optionReducer;
  bipartition::OptionBackbone optionBackone;
  bipartition::OptionDac optionDac;
};
}  // namespace bipe