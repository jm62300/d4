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
                                               SpecManager &s)
{
  std::string meth = vm["partitioning-heuristic"].as<std::string>();
  if(meth == "none") return new PartitioningHeuristicNone();
  if(meth == "bipartition") return new PartitioningHeuristicNone();
  return NULL;
} // makePartitioningHeuristic

}
