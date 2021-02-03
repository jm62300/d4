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
#pragma once
#include <vector>
#include <functional>
#include <boost/program_options.hpp>

#include "src/hyperGraph/HyperGraph.hpp"

namespace d4
{
namespace po = boost::program_options;
class PartitionerManager
{
 public:
  
  static PartitionerManager *makePartitioner(po::variables_map &vm,
                                             unsigned maxNodes,
                                             unsigned maxEdges,
                                             unsigned maxSumEdgeSize);

  virtual ~PartitionerManager() {}

  /**
     Partitioner takes as input an hypergraph given in an array that follows:
     - hypergraph, the given hyper graph
     - flags[i] is 0 if the clause must be ignored.
   */
  virtual void computePartition(HyperGraph &hypergraph,
                                std::function<bool(int)> isAccepted,
                                std::vector<int> &partition) = 0;
};
} // d4
