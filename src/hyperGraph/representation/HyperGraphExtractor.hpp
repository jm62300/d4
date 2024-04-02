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
#include <vector>

#include "HyperGraph.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"

namespace d4 {
class HyperGraphExtractor {
 public:
  virtual ~HyperGraphExtractor() {}

  virtual void constructHyperGraph(SpecManagerCnf &om,
                                   std::vector<Var> &component,
                                   std::vector<Var> &equivClass,
                                   std::vector<std::vector<Var>> &equivVar,
                                   bool reduceFormula,
                                   std::vector<Var> &considered,
                                   HyperGraph &hypergraph) = 0;

  virtual void extractCutFromHyperGraph(HyperGraph &hypergraph,
                                        std::vector<Var> &considered,
                                        std::vector<int> &partition,
                                        std::vector<int> &cutSet) = 0;

#if 0
  /**
Associate for each variable in the component an equivalence class.

@pararm[in] eqManager, the equivalence manager.
@param[in] solver, the SAT solver used in the equivalence manager.
@param[in] component, the set of variables of the component we want to cut.
@param[out] unitEquiv, the set of unit literals we find out.
@param[out] equiClass, the equivalence class we computed (we suppose that
the verctor is large enough and then we do not allocate).
*/
  void PartitioningHeuristic::computeEquivClass(
      EquivExtractor &eqManager, WrapperSolver &solver,
      std::vector<Var> &component, std::vector<Lit> &unitEquiv,
      std::vector<Var> &equivClass, std::vector<std::vector<Var>> &equivVar) {
    for (auto &v : component) {
      assert(equivClass.size() >= (unsigned)v);
      equivClass[v] = v;
    }

    eqManager.searchEquiv(solver, component, equivVar);
    solver.whichAreUnits(component, unitEquiv);

    for (auto &c : equivVar) {
      Var vi = c.back();
      for (auto &v : c) equivClass[v] = vi;
    }
  }  // computeEquivclass
#endif
};
}  // namespace d4
