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
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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
                                                 WrapperSolver &ws)
{
  unsigned options = vm["partitioning-heuristic-options"].as<unsigned>();
  std::string meth = vm["partitioning-heuristic"].as<std::string>();  
  if(meth == "none") return new PartitioningHeuristicNone();
  
  std::string in = vm["input"].as<std::string>();
  std::string extension = in.substr(in.find_last_of(".") + 1);
  if(extension == "cnf" || extension == "dimacs")
  {    
    if(meth == "bipartition")
      return new PartitioningHeuristicBipartite(ws, s, options);
    return NULL;
  }
  
  return NULL;
} // makePartitioningHeuristic

}
