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

#include "HyperGraphExtractor.hpp"

#include "cnf/HyperGraphExtractorCnfDual.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {

/**
 * @brief HyperGraphExtractor::makeHyperGraphExtractor implementation.
 */
HyperGraphExtractor *HyperGraphExtractor::makeHyperGraphExtractor(
    const HyperGraphExtractorMethod &method, const ProblemInputType &inType) {
  switch (inType) {
    case PB_CNF:
      switch (method) {
        case HYPER_GRAPH_DUAL:
          return new HyperGraphExtractorCnfDual();
        default:
          break;
      }
    default:
      std::cerr << "Only CNF formula can be considered, all the other type are "
                   "not supported yet.\n";
      exit(ERROR_BAD_TYPE_PROBLEM);
  }

  throw(FactoryException("Cannot create a HyperGraphExtractor", __FILE__,
                         __LINE__));
}  // makeHyperGraphExtractor
}  // namespace d4
