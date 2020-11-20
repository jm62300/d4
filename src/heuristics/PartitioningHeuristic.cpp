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
#include "cnf/PartitioningHeuristicBipartite.hpp"

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
  out << "c [CONSTRUCTOR] Paritioner manager: " << meth << " " << inType << " "
      << "reduceFormula(" << reduceFormula << ") "
      << "equivSimp(" << equivSimp << ")\n";
  
  
  if(inType == "cnf" || inType == "dimacs")
  {    
    if(meth == "bipartition") return new PartitioningHeuristicBipartite(vm, ws, s);
  }

  throw (FactoryException("Cannot create a PartitioningHeuristic",__FILE__, __LINE__));
} // makePartitioningHeuristic

}
