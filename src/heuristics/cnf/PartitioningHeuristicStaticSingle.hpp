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

#include "PartitioningHeuristicStatic.hpp"

namespace d4
{
class PartitioningHeuristicStaticSingle : public PartitioningHeuristicStatic
{
 protected:
  const unsigned LIMIT = 10;

  // to store the hypergraph, and then avoid reallocated memory.
  HyperGraph m_hypergraph;
  HyperGraphExtractor *m_hypergraphExtractor;

  
  void distributePartition(
      std::vector<std::vector<unsigned> > &hypergraph,
      std::vector<int> &partition,
      std::vector<unsigned> &mappingEdge,
      std::vector<Var> &mappingVar,
      std::vector<Strata> &stack,
      unsigned &level);


  void splitWrtPartition(
      HyperGraph &hypergraph,
      std::vector<int> &partition,
      std::vector<unsigned> &mappingEdge,
      std::vector<unsigned> &cutSet,
      std::vector<unsigned> &indicesFirst,
      std::vector<unsigned> &indicesSecond);
  

  void assignLevel(
      std::vector<std::vector<unsigned> > &hypergraph,
      unsigned idFather,
      std::vector<unsigned> &indices,
      std::vector<Var> &mappingVar,
      unsigned &level);

  void computeDecomposition(
      std::vector<Var> &component,
      std::vector<Var> &equivClass,
      std::vector< std::vector<Var> > &equivVar,
      std::vector<unsigned> &bucketNumber);

  void saveHyperGraph(std::vector<std::vector<unsigned> > &savedHyperGraph);
  
  void setHyperGraph(
      std::vector<std::vector<unsigned> > &savedHyperGraph,
      std::vector<unsigned> &indices,
      HyperGraph &hypergraph);

    virtual void setBucketLevelFromEdges(
      std::vector<std::vector<unsigned> > &hypergraph,
      std::vector<unsigned> &indices,
      std::vector<int> &mapping,
      unsigned level){}

  virtual void setCutSetBucketLevelFromEdges(
      std::vector<std::vector<unsigned> > &hypergraph,
      std::vector<int> &partition,
      std::vector<unsigned> &indices,
      std::vector<int> &mapping,
      unsigned level)
  {
    setBucketLevelFromEdges(hypergraph, indices, mapping, level);
  }
  
 public:
  PartitioningHeuristicStaticSingle(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om);

  PartitioningHeuristicStaticSingle(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om,                                 
      int nbClause,
      int nbVar,
      int sumSize);

  virtual ~PartitioningHeuristicStaticSingle();

  void computeCutSet(
      std::vector<Var> &component,
      std::vector<Var> &cutSet);

};
} // d4
