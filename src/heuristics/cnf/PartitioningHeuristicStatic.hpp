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

#include <cstdint>
#include <vector>
#include <boost/program_options.hpp>

#include "src/solvers/WrapperSolver.hpp"
#include "src/utils/EquivExtractor.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/hyperGraph/HyperGraphExtractorDual.hpp"

#include "../PartitioningHeuristic.hpp"

namespace d4
{
namespace po = boost::program_options;
class PartitioningHeuristicStatic : public PartitioningHeuristic
{
 private:
  WrapperSolver &m_s;  
  SpecManagerCnf &m_om;
  EquivExtractor m_em;
  PartitionerManager *m_pm;

  // to store the hypergraph, and then avoid reallocated memory.
  HyperGraph m_hypergraph;
  HyperGraphExtractor *m_hypergraphExtractor;

  unsigned m_nbVar;
  unsigned m_nbClause;  
  bool m_isInitialized;
  std::vector<unsigned> m_bucketNumber;
  std::vector<unsigned> m_mapVar;

  // options:
  bool m_reduceFormula; 
  bool m_equivSimp;

  const unsigned LIMIT = 10;


  /**
     Save the current hyper graph.

     @param[out] savedHyperGraph, the structure where is saved the graph.
   */
  inline void saveHyperGraph(
      std::vector<std::vector<unsigned> > &savedHyperGraph)
  {
    for(auto edge : m_hypergraph)
    {
      savedHyperGraph.push_back(std::vector<unsigned>());
      std::vector<unsigned> &tmp = savedHyperGraph.back();
      for(auto v : edge) tmp.push_back(v);
    }
  } // savedHyperGraph


  /**
     Set the hyper graph regarding the given set of variables and the saved
     hyper graph.

     @param[in] savedHyperGraph, the current hyper graph.
     @param[in] indices, the current set of edges' indices.
     @param[out] hypergraph, the computed hyper graph.
   */
  inline void setHyperGraph(
      std::vector<std::vector<unsigned> > &savedHyperGraph,
      std::vector<unsigned> &indices,
      HyperGraph &hypergraph)
  {
    unsigned *edges = hypergraph.getEdges();
    hypergraph.setSize(0);

    for(auto idxEdge : indices)
    {
      std::vector<unsigned> &tmp = savedHyperGraph[idxEdge];
      *edges = tmp.size();
      for(unsigned i = 0 ; i<tmp.size() ; i++) edges[i + 1] = tmp[i];      
      edges += *edges + 1;
      hypergraph.incSize();
    }
  } // setHyperGraph
  

  void distributePartition(
      std::vector<unsigned> &indicesFirst,
      std::vector<unsigned> &indicesSecond,
      std::vector<Var> &mappingVar,
      bool cutIsEmpty,
      std::vector< std::vector<unsigned> > &stack,
      unsigned &level);
  
  
 protected:
  void computeDecomposition(
      std::vector<Var> &component,
      std::vector<Var> &equivClass,
      std::vector< std::vector<Var> > &equivVar,
      std::vector<unsigned> &bucketNumber);
  
  void init();
  
 public:
  PartitioningHeuristicStatic(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om);

  PartitioningHeuristicStatic(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om,                                 
      int nbClause,
      int nbVar,
      int sumSize);

  
  ~PartitioningHeuristicStatic();
  
  void computeCutSet(
      std::vector<Var> &component,
      std::vector<Var> &cutSet);
};
} // d4
