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

#include "OptionTreeDecompositionFlowCutter.hpp"

#include "src/treeDecomposition/TreeDecompositionTreeWidth.hpp"

namespace d4 {
/**
 * @brief OptionTreeDecompositionFlowCutter::OptionTreeDecompositionFlowCutter
 * implementation.
 */
OptionTreeDecompositionFlowCutter::OptionTreeDecompositionFlowCutter(
    TreeDecompositionerMethod treeDecompositionTool,
    GraphExtractorMethod graphExtractorMethod, bool useSimpGraphExtractor)
    : m_treeDecompositionTool(treeDecompositionTool),
      m_graphExtractorMethod(graphExtractorMethod),
      m_useSimpGraphExtractor(useSimpGraphExtractor) {
  m_treeDecompositionMethod = TREE_DECOMP_TREE_WIDTH;
}  // constructor

/**
 * @brief OptionTreeDecompositionFlowCutter::display implementation.
 */
void OptionTreeDecompositionFlowCutter::display(std::ostream& out) const {
}  // display

/**
 * @brief OptionTreeDecompositionFlowCutter::createTreeDecomposition
 * implementation.
 */
TreeDecomposition* OptionTreeDecompositionFlowCutter::createTreeDecomposition(
    const ProblemInputType& inType) {
  return new TreeDecompositionTreeWidth(m_treeDecompositionTool,
                                        m_graphExtractorMethod, inType,
                                        m_useSimpGraphExtractor);
}  // createTreeDecomposition

}  // namespace d4