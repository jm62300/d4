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

#include <iostream>

#include "PrimalCutSelector.hpp"

namespace d4 {

/**
 * @brief Recursive bisection variant of PrimalCutSelector.
 *
 * After the first cut, the two residual clause groups (part0, part1) are
 * themselves partitioned recursively.  The F_easy returned is the union of all
 * cut-clause sets collected across the bisection tree up to @p maxDepth levels.
 *
 * More depth → more cubes enumerated from F_easy, but each residual problem
 * presented to the full counter is smaller and more decomposed.
 */
class IterativePrimalCutSelector : public PrimalCutSelector {
 private:
  unsigned m_maxDepth;

  void selectRecursive(const parser::Formula& formula,
                       const std::vector<unsigned>& clauseIndices,
                       unsigned depth, std::vector<unsigned>& result);

 public:
  explicit IterativePrimalCutSelector(unsigned maxDepth = 3,
                                      std::ostream& out = std::cout)
      : PrimalCutSelector(out), m_maxDepth(maxDepth) {}

  std::vector<unsigned> select(const parser::Formula& formula) override;
};

}  // namespace d4
