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
  partitionerName = config.partitionerName;
  treeDecompositionMethod = config.treeDecompositionMethod;
  treeDecompositionerMethod = config.treeDecompositionerMethod;
  hyperGraphExtractorMethod = config.hyperGraphExtractorMethod;
  graphExtractorMethod = config.graphExtractorMethod;
  useSimpGraphExtractor = config.useSimpGraphExtractor;
}  // constructor.

/**
 * @brief Redefinition of <<
 */
std::ostream& operator<<(std::ostream& out,
                         const OptionPartialOrderHeuristic& dt) {
  out << " Option Partitioning Heuristic:"
      << " method("
      << PartialOrderMethodManager::getPartialOrderMethod(dt.partialOrderMethod)
      << ")"
      << " partitioner name("
      << PartitionerNameManager::getPartitionerName(dt.partitionerName) << ")"
      << " tree decomposition method("
      << TreeDecompositionMethodManager::getTreeDecompositionMethod(
             dt.treeDecompositionMethod)
      << ")"
      << " tree decompositioner method("
      << TreeDecompositionerMethodManager::getTreeDecompositionerMethodManager(
             dt.treeDecompositionerMethod)
      << ")"
      << " hyper graph representation("
      << HyperGraphExtractorMethodManager::getHyperGraphExtractorMethodManager(
             dt.hyperGraphExtractorMethod)
      << ")"
      << " graph representation("
      << GraphExtractorMethodManager::getGraphExtractorMethodManager(
             dt.graphExtractorMethod)
      << ")"
      << " graph extractor simplification(" << dt.useSimpGraphExtractor << ")";
  return out;
}  // <<

}  // namespace d4