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
#include "src/hyperGraph/partitioner/PartitionerManager.hpp"
#include "src/hyperGraph/treeDecomposition/TreeDecomposition.hpp"

namespace d4 {

class ConfigurationPartitioningHeuristic;

enum PartitioningMethod { PARTITIONING_TREE_DECOMP, PARTITIONING_NONE };

class PartitioningMethodManager {
 public:
  static std::string getPartitioningMethod(const PartitioningMethod& m) {
    if (m == PARTITIONING_TREE_DECOMP) return "tree decomposition dual";
    if (m == PARTITIONING_NONE) return "none";

    throw(FactoryException("Paritioning method type unknown", __FILE__,
                           __LINE__));
  }  // getPartitioningMethod

  static PartitioningMethod getPartitioningMethod(const std::string& m) {
    if (m == "tree-decomposition") return PARTITIONING_TREE_DECOMP;
    if (m == "none") return PARTITIONING_NONE;

    throw(FactoryException("Paritioning method unknown", __FILE__, __LINE__));
  }  // getPartitioningMethod
};

class OptionPartitioningHeuristic {
 public:
  PartitioningMethod partitioningMethod = PARTITIONING_TREE_DECOMP;
  PartitionerName partitionerName = PARTITIONER_PATOH;
  TreeDecompositionMethod treeDecompositionMethod = TREE_DECOMP_PARTITION;

  /**
   * @brief Construct a new Option Partitioning Heuristic object with the
   * default configuration.
   *
   */
  OptionPartitioningHeuristic();

  /**
   * @brief Construct a new Option Partitioning Heuristic object with the given
   * configuration.
   *
   * @param config is the configuration we want to use.
   */
  OptionPartitioningHeuristic(const ConfigurationPartitioningHeuristic& config);

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionPartitioningHeuristic& dt) {
    out << " Option Paritioning Heuristic:"
        << " method("
        << PartitioningMethodManager::getPartitioningMethod(
               dt.partitioningMethod)
        << ")"
        << " partitioner name("
        << PartitionerNameManager::getPartitionerName(dt.partitionerName)
        << ")";
    return out;
  }  // <<
};
}  // namespace d4