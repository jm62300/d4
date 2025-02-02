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

#include "OptionTreeDecomposition.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/representation/hypergraph/HyperGraphExtractor.hpp"

namespace d4 {
class OptionTreeDecompositionBiPartition : public OptionTreeDecomposition {
 private:
  PartitionerName m_partitionerName;
  HyperGraphExtractorMethod m_hyperGraphExtractorMethod;

 protected:
  /**
   * @brief Display information about the bipartition based approach used to
   * compute the tree decomposition.
   *
   * @param out The output stream to which the object is printed.
   */
  void display(std::ostream& out) const override;

 public:
  /**
   * @brief Constructs an OptionTreeDecompositionBiPartition object, a
   * specialized type of OptionTreeDecomposition.
   *
   * This constructor initializes the method used for recursively partitioning
   * the hypergraph and the method used to extract the hypergraph from the CNF
   * formula.
   *
   * @param partitionerName The method used for recursively partitioning the
   *        hypergraph.
   * @param hyperGraphExtractorMethod The method used to construct the
   * hypergraph from the CNF formula.
   */
  OptionTreeDecompositionBiPartition(
      PartitionerName partitionerName,
      HyperGraphExtractorMethod hyperGraphExtractorMethod);

  /**
   * @brief Creates a TreeDecompositionPartition object.
   *
   * This function overrides the base class method to construct and return
   * a TreeDecomposition object based on the partitioning strategy specified
   * in the current instance.
   *
   * @param[in] inType gives information about the type of formula under
   * consideration.
   *
   * @return A pointer to a TreeDecomposition instance created using the
   * configured partitioning method.
   */
  TreeDecomposition* createTreeDecomposition(
      const ProblemInputType& inType) override;
};
}  // namespace d4