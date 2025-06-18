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

#include "TreeDecomposition.hpp"
#include "src/representation/graph/GraphExtractor.hpp"
#include "src/treeDecompositioner/TreeDecompositioner.hpp"

namespace d4 {
class TreeDecompositionTreeWidth : public TreeDecomposition {
 private:
  TreeDecompositioner *m_treeDecompositioner = NULL;
  GraphExtractor *m_graphExtractor = NULL;
  unsigned m_budget;
  unsigned m_seed;
  bool m_verbose;

 public:
  /**
   * @brief Construct a new object regarding the given options.
   *
   * @param treeDecompositionerMethod is the tree decompositioner we will use.
   * @param graphExtractorMethod is the graph extractor we will use.
   * @param pbType is the type of problem we are handling (CNF, Circuit, ...).
   * @param simplication is set to true if the graph extractor use some kind of
   * simplification.
   * @param budget is a time in second the method is run.
   * @param seed is the seed use if the method use random numbers.
   * @param verbose controls the verbosity.
   */
  TreeDecompositionTreeWidth(
      const TreeDecompositionerMethod &treeDecompositionerMethod,
      const GraphExtractorMethod &graphExtractorMethod,
      const ProblemInputType pbType, bool simplication, unsigned budget,
      unsigned seed, bool verbose);

  /**
   * @brief Destroy the Tree Decomposition Cnf Tree Width object
   */
  ~TreeDecompositionTreeWidth();

  /**
   * @brief Compute a tree decomposition on a CNF using a tool that compute
   * a tree decomposition.
   *
   * @param[in] om gives information about the CNF formula.
   *
   * @return the computed decomposition.
   */
  TreeDecomp *computeDecomposition(FormulaManager &om) override;
};
}  // namespace d4