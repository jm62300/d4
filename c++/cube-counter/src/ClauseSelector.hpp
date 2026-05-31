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

#include <vector>

#include "ParserDimacs.hpp"

namespace d4 {

/**
 * @brief Abstract interface for selecting a subset of clauses to form F_easy.
 *
 * The cube-and-count strategy compiles F_easy into a decision-DNNF, enumerates
 * its partial models (cubes), and counts models of the full formula conditioned
 * on each cube.  The quality of the decomposition depends entirely on which
 * clauses end up in F_easy, which is the responsibility of implementors.
 */
class ClauseSelector {
 public:
  virtual ~ClauseSelector() = default;

  /**
   * @brief Select clause indices to include in F_easy.
   *
   * @param formula The full input formula.
   * @return Indices into formula.clauses of the clauses forming F_easy.
   */
  virtual std::vector<unsigned> select(const parser::Formula& formula) = 0;
};

}  // namespace d4
