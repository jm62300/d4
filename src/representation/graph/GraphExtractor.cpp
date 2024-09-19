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

#include "GraphExtractor.hpp"

#include "circuit/GraphExtractorCircuitPrimal.hpp"
#include "cnf/GraphExtractorCnfPrimal.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {

/**
 * @brief GraphExtractor::makeGraphExtractor implementation.
 */
GraphExtractor *GraphExtractor::makeGraphExtractor(
    const GraphExtractorMethod &method, bool simplication,
    const ProblemInputType &inType) {
  switch (inType) {
    case PB_CIRC:
      switch (method) {
        case GRAPH_PRIMAL:
          return new GraphExtractorCircuitPrimal(simplication);
      }
    case PB_CNF:
      switch (method) {
        case GRAPH_PRIMAL:
          return new GraphExtractorCnfPrimal(simplication);
      }
    case PB_TCNF:
    case PB_QBF:
      std::cerr << "This type of formula is not handle yet.\n";
      exit(ERROR_BAD_TYPE_PROBLEM);
    case PB_NONE:
      std::cerr << "This type none is not supported.\n";
      exit(ERROR_BAD_TYPE_PROBLEM);
  }

  throw(FactoryException("Cannot create GraphExtractor", __FILE__, __LINE__));
}  // makeGraphExtractor

}  // namespace d4