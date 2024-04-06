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

#include "HyperGraphExtractorCnfDual.hpp"

#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {

/**
 * @brief HyperGraphExtractorCnfDual::constructHyperGraph implementation.
 */
InfoHyperGraph HyperGraphExtractorCnfDual::constructHyperGraph(
    SpecManager &om, std::vector<Var> &component, HyperGraph &hypergraph) {
  SpecManagerCnf &tmp = static_cast<SpecManagerCnf &>(om);
  for (auto &v : component) {
    // collect the edge.
    unsigned edgeData[tmp.getNbClause(v)];
    unsigned size = 0;

    for (auto &l : {Lit::makeLitFalse(v), Lit::makeLitTrue(v)}) {
      IteratorIdxClause listIdx = tmp.getVecIdxClauseNotBin(l);
      for (int *ptr = listIdx.start; ptr != listIdx.end; ptr++)
        edgeData[size++] = *ptr;
    }

    // add the hyperedge.
    hypergraph.addEdge({(unsigned)v, size, edgeData});
  }

  return {dynamic_cast<SpecManagerCnf &>(om).getNbVariable(),
          dynamic_cast<SpecManagerCnf &>(om).getNbClause(),
          dynamic_cast<SpecManagerCnf &>(om).getSumSizeClauses()};
}  // constructHyperGraph

}  // namespace d4
