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

#include "../PartitioningHeuristic.hpp"

namespace d4
{
namespace po = boost::program_options;
class PartitioningHeuristicBipartiteDual : public PartitioningHeuristic
{
 private:
  WrapperSolver &m_s;  
  SpecManagerCnf &m_om;
  EquivExtractor m_em;
  PartitionerManager *m_pm;
  
  std::vector<bool> m_markedVar;
  std::vector<bool> m_useLessVariable;
  std::vector<bool> m_markedClauses;
  std::vector<uint64_t> m_hashEdges;
  std::vector<int> m_mapVar;
  std::vector<unsigned> m_idxClauses;
  std::vector<Var> m_equivClass;
  std::vector<int> m_partition;

  // to store the hypergraph, and then avoir reallocated memory.
  unsigned *m_hypergraph;
  unsigned m_hypergraphCapacity;
  unsigned m_hypergraphSize;

  unsigned m_nbVar;
  unsigned m_nbClause;

  // options is given in the constructor and its bytes set the following:
  bool m_reduceFormula; 
  bool m_equivSimp;     


  void constructHyperGraph(std::vector<Var> &component,
                           std::vector<Var> &equivClass,
                           std::vector< std::vector<Var> > &equivVar,
                           std::vector<Var> &considered);
  
  void computeEquivClass(std::vector<Var> &component,
                         std::vector<Lit> &unitEquiv,
                         std::vector<Var> &equivClass,
                         std::vector< std::vector<Var> > &equivVar);
  
  void clashHyperEdgeIndex(std::vector<int> &partition,
                           std::vector<unsigned> &indices);

  void extractCutFromHyperGraph(std::vector<Var> &considered,
                                std::vector<int> &partition,
                                std::vector<int> &cutSet);
  
  void displayHyperGraph(unsigned *hypergraph, unsigned size);
  
 public:
  PartitioningHeuristicBipartiteDual(po::variables_map &vm,
                                       WrapperSolver &s,
                                       SpecManager &om);

  PartitioningHeuristicBipartiteDual(po::variables_map &vm,
                                       WrapperSolver &s,
                                       SpecManager &om,                                 
                                       int nbClause,
                                       int nbVar,
                                       int sumSize);

  
  ~PartitioningHeuristicBipartiteDual();
  
  void computeCutSet(std::vector<Var> &component,
                     std::vector<Var> &cutSet);
};
} // d4
