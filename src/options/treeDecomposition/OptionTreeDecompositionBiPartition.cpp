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

#include "OptionTreeDecompositionBiPartition.hpp"

#include "src/treeDecomposition/TreeDecompositionPartition.hpp"

namespace d4 {
/**
 * @brief OptionTreeDecompositionBiPartition::OptionTreeDecompositionBiPartition
 * implementation.
 */
OptionTreeDecompositionBiPartition::OptionTreeDecompositionBiPartition(
    PartitionerName partitionerName,
    HyperGraphExtractorMethod hyperGraphExtractorMethod)
    : m_partitionerName(partitionerName),
      m_hyperGraphExtractorMethod(hyperGraphExtractorMethod) {
  m_treeDecompositionMethod = TREE_DECOMP_PARTITION;
}  // constructor

/**
 * @brief OptionTreeDecompositionBiPartition::display implementation.
 */
void OptionTreeDecompositionBiPartition::display(std::ostream& out) const {
}  // display

/**
 * @brief OptionTreeDecompositionBiPartition::createTreeDecomposition
 * implementation.
 */
TreeDecomposition* OptionTreeDecompositionBiPartition::createTreeDecomposition(
    const ProblemInputType& inType) {
  return new TreeDecompositionPartition(m_partitionerName,
                                        m_hyperGraphExtractorMethod, inType);
}  // createTreeDecomposition

}  // namespace d4