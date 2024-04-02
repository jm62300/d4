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

#include "HyperGraphExtractorDual.hpp"

#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {

void HyperGraphExtractorDual::constructHyperGraph(SpecManagerCnf &om,
                                                  std::vector<Var> &component,
                                                  HyperGraph &hypergraph) {
  for (auto &v : component) {
    // collect the edge.
    unsigned edgeData[om.getNbClause(v)];
    unsigned size = 0;

    for (auto &l : {Lit::makeLitFalse(v), Lit::makeLitTrue(v)}) {
      IteratorIdxClause listIdx = om.getVecIdxClauseNotBin(l);
      for (int *ptr = listIdx.start; ptr != listIdx.end; ptr++)
        edgeData[size++] = *ptr;
    }

    // add the hyperedge.
    hypergraph.addEdge({(unsigned)v, size, edgeData});
  }
}  // constructHyperGraph

}  // namespace d4
