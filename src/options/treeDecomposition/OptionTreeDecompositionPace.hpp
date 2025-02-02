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
#include "src/representation/graph/GraphExtractor.hpp"
#include "src/treeDecompositioner/TreeDecompositioner.hpp"

namespace d4 {
class OptionTreeDecompositionPace : public OptionTreeDecomposition {
 private:
  TreeDecompositionerMethod m_treeDecompositionTool;
  GraphExtractorMethod m_graphExtractorMethod;
  bool m_useSimpGraphExtractor;

 protected:
  /**
   * @brief Display information about the tree-decomposition tool used to
   * compute the tree decomposition.
   *
   * @param out The output stream to which the object is printed.
   */
  void display(std::ostream& out) const override;

 public:
  /**
   * @brief Constructs an OptionTreeDecompositionPace object, which is a
   * specialized type of OptionTreeDecomposition.
   *
   * This constructor initializes the tree decomposition method, the graph
   * extraction method, and specifies whether to apply simplifications to the
   * extracted graph before computing its tree decomposition.
   *
   * @param treeDecompositionerMethod The method used to compute the tree
   *        decomposition.
   * @param graphExtractorMethod The method used to extract the graph from the
   *        CNF formula.
   * @param useSimpGraphExtractor A boolean indicating whether to apply
   *        simplifications to the extracted graph before processing.
   */
  OptionTreeDecompositionPace(
      TreeDecompositionerMethod treeDecompositionerMethod,
      GraphExtractorMethod graphExtractorMethod, bool useSimpGraphExtractor);

  /**
   * @brief Creates a TreeDecompositionPartition object.
   *
   * This function overrides the base class method to construct and return
   * a TreeDecomposition object based on a tree decomposition tool specified
   * in the current instance.
   *
   * @param[in] inType gives information about the type of formula under
   * consideration.
   *
   * @return A pointer to a TreeDecomposition instance created using the
   * configured tree decomposition method.
   */
  TreeDecomposition* createTreeDecomposition(
      const ProblemInputType& inType) override;
};
}  // namespace d4