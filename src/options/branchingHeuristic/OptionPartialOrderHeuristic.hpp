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
#include "src/heuristics/partialOrder/PartialOrderHeuristic.hpp"
#include "src/options/treeDecomposition/OptionTreeDecomposition.hpp"

namespace d4 {

class ConfigurationPartialOrderHeuristic;
enum PartialOrderHeuristicMethod : char;

class OptionPartialOrderHeuristic {
  /**
   * @brief Construct a new Option Partitioning Heuristic object with the
   * default configuration.
   *
   */
  OptionPartialOrderHeuristic();

 public:
  PartialOrderHeuristicMethod partialOrderMethod;
  OptionTreeDecomposition* optionTreeDecomposition = NULL;
  // is set if partialOrderMethod is PARTIAL_ORDER_GIVEN
  std::vector<double> givenOrder;
  double scaleFactor;

  /**
   * @brief Destroy the object.
   */
  ~OptionPartialOrderHeuristic();

  /**
   * @brief Construct a new Option Partitioning Heuristic object with the given
   * configuration.
   *
   * @param config is the configuration we want to use.
   */
  OptionPartialOrderHeuristic(const ConfigurationPartialOrderHeuristic& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionPartialOrderHeuristic& dt);
};
}  // namespace d4