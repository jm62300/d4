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

#include "PhaseSelectorManager.hpp"
#include "PhaseSelectorStatic.hpp"
#include "PhaseSelectorNone.hpp"

namespace d4
{

/**
   Constructor.

   @param[in] staticPartitioner, give the partitioner used.
 */
PhaseSelectorManager::PhaseSelectorManager(
    PartitioningHeuristicStatic *staticPartitioner)
{
  m_staticPartitioner = staticPartitioner;
} // constructor


/**
   Create a selector manager regarding the given options.

   @param[in] limit, the limit number of variables before switching.
   @param[in] dynamicPhase, if we switch dynamically.
   @param[in] bucketNumber, the computed partition.
   
   \return a selector manager used to decide if we want to switch between the
   static decomposition to the dynamic one.
 */
PhaseSelectorManager *PhaseSelectorManager::makePhaseSelectorManager(
    po::variables_map &vm,
    PartitioningHeuristicStatic *staticPartitioner)
{
  int limitPhase = vm["partitioning-heuristic-bipartite-phase-static"].as<int>();
  bool dynamicPhase = vm["partitioning-heuristic-bipartite-phase-dynamic"].as<bool>();
  std::string phase = vm["partitioning-heuristic-bipartite-phase"].as<std::string>();
  
  if(phase == "none" || (limitPhase <= 0 && !dynamicPhase))
    return new PhaseSelectorNone(staticPartitioner);  
  return new PhaseSelectorStatic(staticPartitioner, limitPhase);
} // makePhaseSelectorManager

} // d4
