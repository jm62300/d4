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

#include "../GraphExtractor.hpp"
#include "src/formulaManager/cnf/CnfManager.hpp"

namespace d4 {
class GraphExtractorCnf : public GraphExtractor {
 public:
  virtual ~GraphExtractorCnf() {}

  /**
   * @brief Try to simplify the CNF (following the variable representation -
   * i.e. we remove information about the literals).
   *
   * @param[out] clauses is the set of clauses we want to simplify.
   */
  void simplication(std::vector<std::vector<int> > &clauses);

  /**
   * @brief Extract the CNF formula (keeping only information about the
   * variables).
   *
   * @param[in] formula is the formula we want to extract.
   * @param[in] component is the set of variables under consideration.
   * @param[out] clauses is the set of computed clauses.
   */
  void extractCnf(CnfManager &formula, std::vector<Var> &component,
                  std::vector<std::vector<int> > &clauses);
};

}  // namespace d4