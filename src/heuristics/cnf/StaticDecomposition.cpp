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

#include "StaticDecomposition.hpp"

namespace d4
{
/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
StaticDecomposition::StaticDecomposition(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om) :
    StaticDecomposition(vm, _s, _om,
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
StaticDecomposition::StaticDecomposition(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om,
    int _nbClause,
    int _nbVar,
    int _sumSize) : m_s(_s), m_om(dynamic_cast<SpecManagerCnf&>(_om))
{
  m_pm = PartitionerManager::makePartitioner(vm, _nbClause, _nbVar, _sumSize);  
  m_em.initEquivExtractor(_nbVar + 1);
  m_nbVar = _nbVar;
  m_nbClause = _nbClause;

  // get the options.
  m_reduceFormula = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  m_equivSimp = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();
  
  m_isInitialized = false;
  m_bucketNumber.resize(m_nbVar, 0);
} // constructor


/**
   Destructor.
*/
StaticDecomposition::~StaticDecomposition()
{
  delete m_pm;
} // destructor


/**
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables.
   @param[out] cutSet, the cut set we compute.
*/
void StaticDecomposition::computeCutSet(
    std::vector<Var> &component,
    std::vector<Var> &cutSet)
{
} // component

} // d4
