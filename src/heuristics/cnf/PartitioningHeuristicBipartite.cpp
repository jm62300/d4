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

#include "PartitioningHeuristicBipartite.hpp"


namespace d4
{

/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
 */
PartitioningHeuristicBipartite::PartitioningHeuristicBipartite(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om,
    unsigned options) :
    PartitioningHeuristicBipartite(vm, _s, _om, options,
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
PartitioningHeuristicBipartite::PartitioningHeuristicBipartite(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om,
    unsigned options,
    int _nbClause,
    int _nbVar,
    int _sumSize) : m_s(_s), m_om(dynamic_cast<SpecManagerCnf&>(_om))
{
  m_pm = PartitionerManager::makePartitioner(vm, _nbVar, _nbClause, _sumSize);
  
  m_em.initEquivExtractor(_nbVar + 1);
  m_nbVar = _nbVar;
  m_nbClause = _nbClause;

  // initialize the vector.
  m_inCurrentComponent.resize(m_nbVar + 1, false);
  m_mapVar.resize(m_nbVar + 1, 0);
  m_markedVar.resize(m_nbVar + 1, false);
  m_equivClass.resize(m_nbVar + 1, 0);
  m_useLessVariable.resize(m_nbVar + 1, false);
  m_partition.resize(m_nbVar + 1, 0);
  m_markedClauses.resize(m_nbClause + 1, false);
  
  // get the options.
  m_reduceFormula = options & 1;
  m_equivSimp = (options>>1) & 1;

  m_hypergraphCapacity = m_nbClause + _sumSize + 1;
  m_hypergraph = new unsigned[m_hypergraphCapacity];
  m_hypergraphSize = 0;
} // constructor


/**
   Destructor.
 */
PartitioningHeuristicBipartite::~PartitioningHeuristicBipartite()
{
  delete[] m_hypergraph;
  delete m_pm;
} // destructor


/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).
   
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
 */
void PartitioningHeuristicBipartite::extractCutFromHyperGraph(
    std::vector<int> &partition,
    std::vector<int> &cutSet)
{
  std::vector<unsigned *> indices;
  clashHyperEdgeIndex(partition, indices);
  
  for(auto &edge : indices)
  {
    for(unsigned i = 0 ; i<*edge ; i++)
    {
      unsigned x = edge[i + 1];
      if(!m_markedVar[x])
      {
        m_markedVar[x] = true;
        cutSet.push_back(x);
      }
    }
  }
  
  for(auto &x : cutSet) m_markedVar[x] = false; // reinit
} // extractCutFromClauses


/**
   Get the clauses we will use in the partitioning algorithm.

   @param[in] component, the set of variables we focus on.
   @param[in] equiClass, the equivalence class.   
 */
void PartitioningHeuristicBipartite::constructHyperGraph(
    std::vector<Var> &component,
    std::vector<Var> &equivClass)
{
  m_hashEdges.resize(0);
  m_hypergraphSize = 0;
  
  // collect the indices of the clauses from the spec manager.
  for(auto &v : component) m_inCurrentComponent[v] = true;
  m_idxClauses.clear();
  m_om.getCurrentClauses(m_idxClauses, m_inCurrentComponent);
  for(auto &v : component) m_inCurrentComponent[v] = false;
  
  // construct the hypergraph.
  unsigned *edge = m_hypergraph;  
  for(auto &idx : m_idxClauses)
  {
    uint64_t hash = 0;
    *edge = 0;
    
    for(auto &l : m_om.getClause(idx))
      if(!m_om.litIsAssigned(l) && !m_markedVar[equivClass[l.var()]])
      {
        hash |= (uint64_t) 1<<(((uint64_t)equivClass[l.var()])&63);        
        m_markedVar[equivClass[l.var()]] = true;
        edge[++(*edge)] = equivClass[l.var()];
      }

    for(unsigned i = 0 ; i<*edge ; i++) m_markedVar[edge[i + 1]] = false;
    if(*edge > 1)
    {
      assert(hash);
      m_hashEdges.push_back(hash);
      edge = &(edge[*edge + 1]);
      m_hypergraphSize++;
    }     
  }

  // remove useless edges.
  if(m_reduceFormula) removeSubsumEdges();
}// collectRelevantIdxClauses


/**
   We remove from the hypergraph the edges that are subsumed by another one.

   @param[out] hypergraph, the list of hyper edges.
*/
void PartitioningHeuristicBipartite::removeSubsumEdges()
{
  unsigned *edge = m_hypergraph;
  for(unsigned i = 0 ; i<m_hypergraphSize ; i++)
  {    
    if(m_hashEdges[i])
    {    
      // mark the varaibles of the current edge.
      for(unsigned j = 0 ; j<*edge ; j++) m_markedVar[edge[1 + j]] = true;

      // visit the other edges to compute those that subsubmed or are subsubmed.
      bool subsumed = false;
      unsigned *kedge = &(edge[*edge + 1]);
      for(unsigned k = i + 1 ; k<m_hypergraphSize ; k++)
      {
        if(m_hashEdges[k])
        {          
          uint64_t inter = m_hashEdges[i] & m_hashEdges[k];
          if(m_hashEdges[i] == inter || m_hashEdges[k] == inter)
          {
            unsigned cpt = 0;
            for(unsigned j = 0 ; j<*kedge ; j++) if(m_markedVar[kedge[1 + j]]) cpt++;
        
            if(cpt == *edge) subsumed = true;     // the current edge is smaller then include
            if(cpt == *kedge) m_hashEdges[k] = 0; // the edges k subsums i
          }
        }

        kedge = &(kedge[*kedge + 1]);
      }

      for(unsigned j = 0 ; j<*edge ; j++) m_markedVar[edge[1 + j]] = false; // reinit
      if(subsumed) m_hashEdges[i] = 0;
    }

    edge = &(edge[*edge + 1]); // progress to the next clause.
  }
} // removeSubsumEdges


/**
   Associate for each variable in the component an equivalence class.

   @param[in] component, the set of variables of the component we want to cut.
   @param[out] unitEquiv, the set of unit literals we find out.
   @param[out] equiClass, the equivalence class we computed (we suppose that the
   verctor is large enough and then we do not allocate).
 */
void PartitioningHeuristicBipartite::computeEquivClass(
    std::vector<Var> &component,
    std::vector<Lit> &unitEquiv,
    std::vector<Var> &equivClass)
{
  assert(equivClass.size() >= m_nbVar);
  for(auto &v : component) equivClass[v] = v;
  if(!m_equivSimp) return;

  std::vector< std::vector<Var> > equivVar;
  m_em.searchEquiv(m_s, component, equivVar);
  m_s.whichAreUnits(component, unitEquiv);
  
  // propagate the equivVar information in equivClass
  for(auto &c : equivVar)
  {
    Var vi = c.back();
    for(auto &v : c) equivClass[v] = vi;             
  }
} // computeEquivClass

/**
   Collect the set of hyper egdes (their indices actually) that are between
   several component.

   @param[in] hypergraph, the hypergraph.
   @param[in] partition, the partition.
   @param[in] indices, the list of edge's indices that clash.
 */
void PartitioningHeuristicBipartite::clashHyperEdgeIndex(
    std::vector<int> &partition,
    std::vector<unsigned *> &indices)
{
  bool clash = false;
  int part = 0;
  unsigned *edge = m_hypergraph;
  for(unsigned i = 0 ; i<m_hypergraphSize ; i++)
  {
    if(m_hashEdges[i])
    {    
      clash = false;
      part = partition[edge[1]];

      for(unsigned j = 1 ; !clash && j<*edge ; j++) clash = part != partition[edge[1 + j]];
      if(clash) indices.push_back(edge);
    }

    edge = &(edge[*edge + 1]); // next clause.
  }
} // clashHyperEdgeIndex

/**
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables.
   @param[out] cutSet, the cut set we compute.
 */
void PartitioningHeuristicBipartite::computeCutSet(
    std::vector<Var> &component,
    std::vector<Var> &cutSet)
{
  // search for equiv class if requiered.
  std::vector<Lit> unitEquiv;
  computeEquivClass(component, unitEquiv, m_equivClass);
  
  // synchronize the SAT solver and the spec manager.
  m_om.preUpdate(unitEquiv);

  // construct the hypergraph
  constructHyperGraph(component, m_equivClass);
  assert(m_hashEdges.size() == m_hypergraphSize);
  m_pm->computePartition(m_hypergraph, m_hypergraphSize, m_hashEdges, m_partition);
  
  // collect the cut.
  extractCutFromHyperGraph(m_partition, cutSet);
  
  m_om.postUpdate(unitEquiv);
} // component
} // d4
