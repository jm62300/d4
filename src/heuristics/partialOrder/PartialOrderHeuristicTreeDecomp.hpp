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

#include <cstdint>
#include <ostream>
#include <vector>

#include "PartialOrderHeuristic.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {

class PartialOrderHeuristicTreeDecomp : public PartialOrderHeuristic {
 protected:
  std::vector<unsigned> m_topologicalOrder;

 public:
  /**
   * @brief Factory to create a tree decomposition partitioner manager.
   *
   * @param options gives the list of options.
   * @param om is the object which deal with the formula.
   * @param s is a SAT solver associate with the formula under consideration.
   * @param out is the output stream.
   * @return a tree decomposition partitioner.
   */
  static PartialOrderHeuristicTreeDecomp *makePartitioningTreeDecomp(
      const OptionPartialOrderHeuristic &options, SpecManager &om,
      WrapperSolver &s, std::ostream &out);

  void computeCutSet(std::vector<Var> &component,
                     std::vector<Var> &cutSet) override;
};
}  // namespace d4
