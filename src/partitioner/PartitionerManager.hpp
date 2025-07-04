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

#include <functional>
#include <vector>

#include "src/exceptions/FactoryException.hpp"
#include "src/representation/hypergraph/HyperGraph.hpp"
#include "src/representation/hypergraph/HyperGraphExtractor.hpp"

namespace d4 {

enum PartitionerName { PARTITIONER_PATOH };

class PartitionerNameManager {
 public:
  static std::string getPartitionerName(const PartitionerName &m) {
    if (m == PARTITIONER_PATOH) return "patoh";
    throw(FactoryException("Partitioner name unknown", __FILE__, __LINE__));
  }  // getPartitionerName

  static PartitionerName getPartitionerName(const std::string &m) {
    if (m == "patoh") return PARTITIONER_PATOH;
    throw(FactoryException("Partitioner name unknown", __FILE__, __LINE__));
  }  // getPartitionerName
};

class PartitionerManager {
 public:
  enum Level { NORMAL, SPEED, QUALITY };

  /**
   * @brief Fatory for creating a partitioner.
   *
   * @param partitioner is the method name.
   * @param infoHyperGraph gives information about the worst case hypergraph.
   * @param out is the stream where information will be printed out.
   *
   * @return a partitioner manager.
   */
  static PartitionerManager *makePartitioner(
      PartitionerName partitioner, const InfoHyperGraph &infoHyperGraph,
      std::ostream &out);

  virtual ~PartitionerManager() {}

  virtual void computePartition(HyperGraph &hypergraph, Level level,
                                std::vector<int> &partition) = 0;
};
}  // namespace d4
