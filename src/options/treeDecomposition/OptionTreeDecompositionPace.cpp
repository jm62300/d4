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

#include "OptionTreeDecompositionPace.hpp"

#include "src/treeDecomposition/TreeDecompositionTreeWidth.hpp"

namespace d4 {
/**
 * @brief OptionTreeDecompositionPace::OptionTreeDecompositionPace
 * implementation.
 */
OptionTreeDecompositionPace::OptionTreeDecompositionPace(
    TreeDecompositionerMethod treeDecompositionTool,
    GraphExtractorMethod graphExtractorMethod, bool useSimpGraphExtractor,
    unsigned budget, unsigned seed, bool verbosity)
    : m_treeDecompositionTool(treeDecompositionTool),
      m_graphExtractorMethod(graphExtractorMethod),
      m_useSimpGraphExtractor(useSimpGraphExtractor),
      m_budget(budget),
      m_seed(seed),
      m_verbosity(verbosity) {
  m_treeDecompositionMethod = TREE_DECOMP_TREE_WIDTH;
}  // constructor

/**
 * @brief OptionTreeDecompositionPace::display implementation.
 */
void OptionTreeDecompositionPace::display(std::ostream& out) const {
  out << "tree-decomp-tool("
      << TreeDecompositionerMethodManager::getTreeDecompositionerMethodManager(
             m_treeDecompositionTool)
      << ") graph-extractor("
      << GraphExtractorMethodManager::getGraphExtractorMethodManager(
             m_graphExtractorMethod)
      << ") simplification(" << m_useSimpGraphExtractor << ") budget("
      << m_budget << ") seed(" << m_seed << ")\n";
}  // display

/**
 * @brief OptionTreeDecompositionPace::createTreeDecomposition
 * implementation.
 */
TreeDecomposition* OptionTreeDecompositionPace::createTreeDecomposition(
    const ProblemInputType& inType) {
  return new TreeDecompositionTreeWidth(
      m_treeDecompositionTool, m_graphExtractorMethod, inType,
      m_useSimpGraphExtractor, m_budget, m_seed, m_verbosity);
}  // createTreeDecomposition

}  // namespace d4