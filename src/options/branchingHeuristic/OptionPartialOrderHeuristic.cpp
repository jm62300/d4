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

#include "OptionPartialOrderHeuristic.hpp"

#include "src/configurations/ConfigurationPartialOrderHeuristic.hpp"
#include "src/options/treeDecomposition/OptionTreeDecompositionBiPartition.hpp"
#include "src/options/treeDecomposition/OptionTreeDecompositionPace.hpp"

namespace d4 {
/**
 * @brief Construct a new Option Partitioning Heuristic object with the
 * default configuration.
 *
 */
OptionPartialOrderHeuristic::OptionPartialOrderHeuristic()
    : OptionPartialOrderHeuristic(ConfigurationPartialOrderHeuristic()) {
}  // constructor.

/**
 * @brief Construct a new Option Partitioning Heuristic object with the given
 * configuration.
 *
 * @param config is the configuration we want to use.
 */
OptionPartialOrderHeuristic::OptionPartialOrderHeuristic(
    const ConfigurationPartialOrderHeuristic& config) {
  partialOrderMethod = config.partialOrderMethod;
  givenOrder = config.givenOrder;
  scaleFactor = config.scaleFactor;

  switch (config.treeDecompositionMethod) {
    case TREE_DECOMP_PARTITION: {
      optionTreeDecomposition = new OptionTreeDecompositionBiPartition(
          config.partitionerName, config.hyperGraphExtractorMethod);
      break;
    }
    case TREE_DECOMP_TREE_WIDTH: {
      optionTreeDecomposition = new OptionTreeDecompositionPace(
          config.treeDecompositionerMethod, config.graphExtractorMethod,
          config.useSimpGraphExtractor, config.budget, config.seed,
          config.verbosity);
      break;
    }
    default:
      assert(0);
  }
}  // constructor.

/**
 * @brief OptionPartialOrderHeuristic::~OptionPartialOrderHeuristic
 * implementation.
 *
 */
OptionPartialOrderHeuristic::~OptionPartialOrderHeuristic() {
  if (optionTreeDecomposition) delete optionTreeDecomposition;
}  // destructor

/**
 * @brief Redefinition of <<
 */
std::ostream& operator<<(std::ostream& out,
                         const OptionPartialOrderHeuristic& dt) {
  out << " Option Partitioning Heuristic:"
      << " method("
      << PartialOrderMethodManager::getPartialOrderMethod(dt.partialOrderMethod)
      << ") " << *dt.optionTreeDecomposition;
  return out;
}  // <<

}  // namespace d4