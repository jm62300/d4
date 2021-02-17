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
#include "src/hyperGraph/HyperGraphExtractor.hpp"

#include "../PartitioningHeuristic.hpp"

namespace d4
{
namespace po = boost::program_options;
class PartitioningHeuristicStatic : public PartitioningHeuristic
{
  struct Strata
  {
    unsigned fatherId;
    std::vector<unsigned> part;
  };
  
 protected:
  WrapperSolver &m_s;  
  SpecManagerCnf &m_om;
  EquivExtractor m_em;
  PartitionerManager *m_pm;

  // to store the hypergraph, and then avoid reallocated memory.
  HyperGraph m_hypergraph;
  HyperGraphExtractor *m_hypergraphExtractor;

  unsigned m_nbVar;
  unsigned m_nbClause;
  unsigned m_maxNbNodes;
  unsigned m_maxNbEdges;

  bool m_isInitialized;
  std::vector<unsigned> m_bucketNumber;
  std::vector<unsigned> m_separtorLevel;

  // options:
  bool m_reduceFormula; 
  bool m_equivSimp;

  const unsigned LIMIT = 10;

  void saveHyperGraph(std::vector<std::vector<unsigned> > &savedHyperGraph);
  
  void setHyperGraph(
      std::vector<std::vector<unsigned> > &savedHyperGraph,
      std::vector<unsigned> &indices,
      HyperGraph &hypergraph);
  

  void distributePartition(
      std::vector<std::vector<unsigned> > &hypergraph,
      std::vector<Var> &cutSet,
      std::vector<unsigned> &indicesFirst,
      std::vector<unsigned> &indicesSecond,
      std::vector<Var> &mappingVar,
      std::vector<Strata> &stack,
      unsigned &level);
  
  
 protected:
  void computeDecomposition(
      std::vector<Var> &component,
      std::vector<Var> &equivClass,
      std::vector< std::vector<Var> > &equivVar,
      std::vector<unsigned> &bucketNumber);
  
  void init();
  
  virtual void setBucketLevelFromEdges(
      std::vector<std::vector<unsigned> > &hypergraph,
      std::vector<unsigned> &indices,
      std::vector<int> &mapping,
      unsigned level){}

 protected:
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

 public:  
  ~PartitioningHeuristicStatic();

  inline std::vector<unsigned> &getBucketNumber(){return m_bucketNumber;}
  inline std::vector<unsigned> &getSeparatorLevel(){return m_separtorLevel;}
  
  static PartitioningHeuristicStatic *makePartitioningHeuristicStatic(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om,
      int nbClause,
      int nbVar,
      int sumSize,
      const std::string &type);
  
  void computeCutSet(
      std::vector<Var> &component,
      std::vector<Var> &cutSet);

  virtual bool isReady() {return true;}
};
} // d4
