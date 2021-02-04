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
   Get the clauses we will use in the partitioning algorithm.

   @param[in] component, the set of variables we focus on.
   @param[in] equiClass, the equivalence class.   
*/
void PartitioningHeuristicBipartitePrimal::constructHyperGraph(
    std::vector<Var> &component,
    std::vector<Var> &equivClass)
{
  m_hashEdges.resize(0);
  m_hypergraph.setSize(0);
  
  // collect the indices of the clauses from the spec manager.  
  m_om.getCurrentClauses(m_idxClauses, component);
  
  // construct the hypergraph.
  unsigned *edge = m_hypergraph.getEdges();
  for(auto &idx : m_idxClauses)
  {
    uint64_t hash = 0;
    *edge = 0;
    
    for(auto &l : m_om.getClause(idx))
    {
      if(!m_om.litIsAssigned(l) && !m_markedVar[equivClass[l.var()]])
      {
        hash |= (uint64_t) 1<<(((uint64_t)equivClass[l.var()])&63);        
        m_markedVar[equivClass[l.var()]] = true;
        edge[++(*edge)] = equivClass[l.var()];
      }
    }

    for(unsigned i = 0 ; i<*edge ; i++) m_markedVar[edge[i + 1]] = false;
    if(*edge > 1)
    {
      assert(hash);
      m_hashEdges.push_back(hash);
      m_hypergraph.incSize();
      edge = &(edge[*edge + 1]);      
    }     
  }

  // remove useless edges.
  if(m_reduceFormula) removeSubsumEdges(m_hypergraph.getEdges(),
                                        m_hypergraph.getSize());
}// collectRelevantIdxClauses


/**
   We remove from the hypergraph the edges that are subsumed by another one.
   The edges are not directly removed but their hash is set to 0 if they are
   removed.

   @param[out] hypergraph, the list of hyper edges.
   @param[in] size, the number of hyperedge.
*/
void PartitioningHeuristicBipartitePrimal::removeSubsumEdges(
    unsigned *hypergraph,
    unsigned size)
{
  unsigned *edge = hypergraph;
  for(unsigned i = 0 ; i<size ; i++)
  {    
    if(m_hashEdges[i])
    {    
      // mark the varaibles of the current edge.
      for(unsigned j = 0 ; j<*edge ; j++) m_markedVar[edge[1 + j]] = true;

      // visit the other edges to compute those that subsubmed or are subsubmed.
      bool subsumed = false;
      unsigned *kedge = &(edge[*edge + 1]);
      for(unsigned k = i + 1 ; k<size ; k++)
      {
        if(m_hashEdges[k])
        {          
          uint64_t inter = m_hashEdges[i] & m_hashEdges[k];
          if(m_hashEdges[i] == inter || m_hashEdges[k] == inter)
          {
            unsigned cpt = 0;
            for(unsigned j = 0 ; j<*kedge ; j++) if(m_markedVar[kedge[1 + j]]) cpt++;
        
            if(cpt == *edge) subsumed = true;          // the current edge is smaller then include
            else if(cpt == *kedge) m_hashEdges[k] = 0; // the edges k subsums i 
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
