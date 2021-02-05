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

#include "PartitioningHeuristicBipartitePrimal.hpp"

namespace d4
{
/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicBipartitePrimal::PartitioningHeuristicBipartitePrimal(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om) :
    PartitioningHeuristicBipartitePrimal(vm, _s, _om,
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
PartitioningHeuristicBipartitePrimal::PartitioningHeuristicBipartitePrimal(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om,
    int _nbClause,
    int _nbVar,
    int _sumSize) : m_s(_s), m_om(dynamic_cast<SpecManagerCnf&>(_om))
{
  m_pm = PartitionerManager::makePartitioner(vm, _nbVar, _nbClause, _sumSize);
  
  m_em.initEquivExtractor(_nbVar + 1);
  m_nbVar = _nbVar;
  m_nbClause = _nbClause;

  // initialize the vector.
  m_mapVar.resize(m_nbVar + 1, 0);
  m_markedVar.resize(m_nbVar + 1, false);
  m_equivClass.resize(m_nbVar + 1, 0);
  m_useLessVariable.resize(m_nbVar + 1, false);
  m_partition.resize(m_nbVar + 1, 0);
  m_markedClauses.resize(m_nbClause + 1, false);
  
  // get the options.
  m_reduceFormula = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  m_equivSimp = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();

  m_hypergraph.init(m_nbClause + _sumSize + 1);
  m_hypergraphExtractor = new HyperGraphExtractorPrimal(m_nbVar, m_nbClause);
} // constructor


/**
   Destructor.
*/
PartitioningHeuristicBipartitePrimal::~PartitioningHeuristicBipartitePrimal()
{
  delete m_hypergraphExtractor;
  delete m_pm;
} // destructor



/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).
   We try to minimize the cut in a greedy fashion.
   
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
*/
void PartitioningHeuristicBipartitePrimal::extractCutFromHyperGraph(
    std::vector<int> &partition,
    std::vector<int> &cutSet)
{
  std::vector<unsigned *> indices;
  clashHyperEdgeIndex(partition, indices);
  
  for(auto &edge : indices)
  {
    int cpt0 = 0, cpt1 = 0;
    for(unsigned i = 0 ; i<*edge ; i++)
    {
      unsigned x = edge[i + 1];
      if(m_markedVar[x]) continue;
      if(partition[x]) cpt1++; else cpt0++;
    }

    int selected = (cpt0 < cpt1) ? 0 : 1;    
    for(unsigned i = 0 ; i<*edge ; i++)
    {
      unsigned x = edge[i + 1];
      if(!m_markedVar[x] && partition[x] == selected)
      {
        m_markedVar[x] = true;
        cutSet.push_back(x);
      }
    }
  }
  
  for(auto &x : cutSet) m_markedVar[x] = false; // reinit
} // extractCutFromClauses


/**
   Collect the set of hyper egdes (their indices actually) that are between
   several component.

   @param[in] hypergraph, the hypergraph.
   @param[in] partition, the partition.
   @param[in] indices, the list of edge's indices that clash.
*/
void PartitioningHeuristicBipartitePrimal::clashHyperEdgeIndex(
    std::vector<int> &partition,
    std::vector<unsigned *> &indices)
{
  bool clash = false;
  int part = 0;
  unsigned *edge = m_hypergraph.getEdges();
  for(unsigned i = 0 ; i<m_hypergraph.getSize() ; i++)
  {
    clash = false;
    part = partition[edge[1]];

    for(unsigned j = 1 ; !clash && j<*edge ; j++) clash = part != partition[edge[1 + j]];
    if(clash) indices.push_back(edge);

    edge = &(edge[*edge + 1]); // next clause.
  }
} // clashHyperEdgeIndex

/**
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables.
   @param[out] cutSet, the cut set we compute.
*/
void PartitioningHeuristicBipartitePrimal::computeCutSet(
    std::vector<Var> &component,
    std::vector<Var> &cutSet)
{
  // search for equiv class if requiered.
  std::vector<Lit> unitEquiv;
  std::vector< std::vector<Var> > equivVar;
  if(m_equivSimp) PartitioningHeuristic::computeEquivClass(
         m_em, m_s, component, unitEquiv, m_equivClass, equivVar);
  else for(auto &v : component) m_equivClass[v] = v;

  // synchronize the SAT solver and the spec manager.
  m_om.preUpdate(unitEquiv);
  
  // construct the hypergraph
  std::vector<Var> considered;
  m_hypergraphExtractor->constructHyperGraph(
      m_om, component, m_equivClass, equivVar, m_reduceFormula, considered, m_hypergraph);
  
  m_pm->computePartition(m_hypergraph, m_partition);
  
  // collect the cut.
  extractCutFromHyperGraph(m_partition, cutSet);
  
  // extend with equivalence literals.
  for(auto &v : cutSet) m_markedVar[v] = true;
  for(auto &v : component)
  {
    if(m_markedVar[v]) continue;
    if(m_markedVar[m_equivClass[v]]) cutSet.push_back(v);
  }  
  for(auto &v : cutSet) m_markedVar[v] = false;
  m_om.postUpdate(unitEquiv);
} // component
} // d4
