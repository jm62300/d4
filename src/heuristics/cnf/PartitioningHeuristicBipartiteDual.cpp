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
#include <bitset>
#include <algorithm>

#include "PartitioningHeuristicBipartiteDual.hpp"

namespace d4
{
/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicBipartiteDual::PartitioningHeuristicBipartiteDual(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om) :
    PartitioningHeuristicBipartiteDual(vm, _s, _om,
                                         dynamic_cast<SpecManagerCnf&>(_om).getNbClause(),
                                         dynamic_cast<SpecManagerCnf&>(_om).getNbVariable(),
                                         dynamic_cast<SpecManagerCnf&>(_om).getSumSizeClauses())
{  
} // constructor


/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicBipartiteDual::PartitioningHeuristicBipartiteDual(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om,
    int nbClause,
    int nbVar,
    int sumSize) :
    PartitioningHeuristicBipartite(vm, om, s, nbClause, nbVar, sumSize)
{
  // initialize the vector.
  m_partition.resize(m_nbClause + 1, 0);

  m_pm = PartitionerManager::makePartitioner(vm, m_nbClause, m_nbVar, sumSize);
  m_hypergraph.init(m_nbVar + m_nbClause + sumSize + 1);  
  m_hypergraphExtractor = new HyperGraphExtractorDual(m_nbVar, m_nbClause);
} // constructor    

/**
   Collect the set of hyper egdes (their indices actually) that are between
   several component.

   @param[in] hypergraph, the hypergraph.
   @param[in] partition, the partition.
   @param[in] indices, the list of edge's indices that clash.
*/
void PartitioningHeuristicBipartiteDual::clashHyperEdgeIndex(
    HyperGraph &hypergraph,
    std::vector<int> &partition,
    std::vector<unsigned> &indices)
{
  bool clash = false;
  int part = 0;
  
  for(auto edge : hypergraph)
  {
    clash = false;
    part = partition[edge[0]];

    for(unsigned j = 1 ; !clash && j<edge.getSize() ; j++)
      clash = part != partition[edge[j]];
    if(clash) indices.push_back(edge.getId());
  }
} // clashHyperEdgeIndex


/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).
   We try to minimize the cut in a greedy fashion.

   @param[in] considered, the label variables for the edges.
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
*/
void PartitioningHeuristicBipartiteDual::extractCutFromHyperGraph(
    std::vector<Var> &considered,
    std::vector<int> &partition,
    std::vector<int> &cutSet)
{
  std::vector<unsigned> indices;
  clashHyperEdgeIndex(m_hypergraph, partition, indices);
  for(auto &i : indices) cutSet.push_back(considered[i]);
} // extractCutFromClauses

} // d4
