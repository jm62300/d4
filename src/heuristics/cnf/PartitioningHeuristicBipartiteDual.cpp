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
  
  m_staticPartitioner = PartitioningHeuristicStatic::makePartitioningHeuristicStatic(
      vm, s, om, nbClause, nbVar, sumSize, "dual");
} // constructor

} // d4
