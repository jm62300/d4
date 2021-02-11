/*
* d4
* Copyright (C) 2020  Univ. Artois & CNRS
* 
* This program is free software: you can redistribute it and/or modify
a* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <bitset>

#include "src/exceptions/FactoryException.hpp"

#include "PartitioningHeuristic.hpp"
#include "PartitioningHeuristicNone.hpp"
#include "cnf/PartitioningHeuristicBipartitePrimal.hpp"
#include "cnf/PartitioningHeuristicBipartiteDual.hpp"
#include "cnf/PartitioningHeuristicStatic.hpp"

namespace d4
{

/**
   Create a partitioner.

   @param[in] vm, the list of options.
   @param[in] s, a view on the problem's structure.

   \return a partioner if the options are ocrrect, NULL otherwise.
 */
PartitioningHeuristic *
PartitioningHeuristic::makePartitioningHeuristic(po::variables_map &vm,
                                                 SpecManager &s,
                                                 WrapperSolver &ws,
                                                 std::ostream &out)
{
  std::string meth = vm["partitioning-heuristic"].as<std::string>();  
  std::string inType = vm["input-type"].as<std::string>();

  
  if(meth == "none")
  {
    out << "c [CONSTRUCTOR] Paritioner manager: " << meth << " " << inType << "\n";
    return new PartitioningHeuristicNone();
  }

  bool reduceFormula = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  bool equivSimp = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();
  bool staticPhase = vm["partitioning-heuristic-bipartite-phase"].as<bool>();
  out << "c [CONSTRUCTOR] Paritioner manager: " << meth << " " << inType << " "
      << "reduceFormula(" << reduceFormula << ") "
      << "equivSimp(" << equivSimp << ") "
      << "staticPhase(" << staticPhase << ")\n";
  
  
  if(inType == "cnf" || inType == "dimacs")
  {    
    if(meth == "bipartition-primal")
      return new PartitioningHeuristicBipartitePrimal(vm, ws, s);
    if(meth == "bipartition-dual")
      return new PartitioningHeuristicBipartiteDual(vm, ws, s);
    if(meth == "decomposition-static")
      return new PartitioningHeuristicStatic(vm, ws, s);
  }

  throw (FactoryException("Cannot create a PartitioningHeuristic",__FILE__, __LINE__));
} // makePartitioningHeuristic


/**
   Associate for each variable in the component an equivalence class.

   @pararm[in] eqManager, the equivalence manager.
   @param[in] solver, the SAT solver used in the equivalence manager.
   @param[in] component, the set of variables of the component we want to cut.
   @param[out] unitEquiv, the set of unit literals we find out.
   @param[out] equiClass, the equivalence class we computed (we suppose that the
   verctor is large enough and then we do not allocate).
*/
void PartitioningHeuristic::computeEquivClass(
    EquivExtractor &eqManager,
    WrapperSolver &solver,
    std::vector<Var> &component,
    std::vector<Lit> &unitEquiv,
    std::vector<Var> &equivClass,
    std::vector< std::vector<Var> > &equivVar)
{
  for(auto &v : component)
  {
    assert(equivClass.size() >= (unsigned) v);
    equivClass[v] = v;
  }
  
  eqManager.searchEquiv(solver, component, equivVar);
  solver.whichAreUnits(component, unitEquiv);
  
  // propagate the equivVar information in equivClass
  for(auto &c : equivVar)
  {
    Var vi = c.back();
    for(auto &v : c) equivClass[v] = vi;             
  }
} // computeEquivclass

}
