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
  m_pm = PartitionerManager::makePartitioner(vm, _nbClause, _nbVar, _sumSize);
  
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
} // constructor


/**
   Destructor.
 */
PartitioningHeuristicBipartite::~PartitioningHeuristicBipartite()
{
  delete m_pm;
} // destructor


/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).

   @param[in] hypergraph, the list of hyper edges.
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
 */
void PartitioningHeuristicBipartite::extractCutFromHyperGraph(
    std::vector< std::vector<unsigned> > &hypergraph,
    std::vector<int> &partition,
    std::vector<int> &cutSet)
{
  std::vector<unsigned> indices;
  clashHyperEdgeIndex(hypergraph, partition, indices);

  int balance = 0;
  for(auto &idx : indices)
  {
    int part = balance >= 0 ? 0 : 1;
    for(auto &x : hypergraph[idx])
    {
      if(!m_markedVar[x] && partition[x] == part)
      {
        m_markedVar[x] = true;
        cutSet.push_back(x);
        if(part) balance++; else balance--;
      }
    }
  }
  
  for(auto &x : cutSet) m_markedVar[x] = false; // reinit
} // extractCutFromClauses


/**
   Get the clauses we will use in the partitioning algorithm.

   @param[in] component, the set of variables we focus on.
   @param[in] equiClass, the equivalence class.
   @param[out] hypergraph, the output hyper graph.
   
 */
void PartitioningHeuristicBipartite::constructHyperGraph(
    std::vector<Var> &component,
    std::vector<Var> &equivClass,
    std::vector< std::vector<unsigned> > &hypergraph)
{
  // collect the indices of the clauses from the spec manager.
  for(auto &v : component) m_inCurrentComponent[v] = true;
  m_idxClauses.clear();
  m_om.getCurrentClauses(m_idxClauses, m_inCurrentComponent);
  for(auto &v : component) m_inCurrentComponent[v] = false;
  
  // construct the hypergraph.
  for(auto &idx : m_idxClauses)
  {
    hypergraph.push_back(std::vector<unsigned>());
    std::vector<unsigned> &next = hypergraph.back();

    for(auto &l : m_om.getClause(idx))
      if(!m_om.litIsAssigned(l) && !m_markedVar[equivClass[l.var()]])
      {
        m_markedVar[equivClass[l.var()]] = true;
        next.push_back(equivClass[l.var()]);
      }

    for(auto &x : next) m_markedVar[x] = false;
    if(next.size() == 1) hypergraph.pop_back();
  }

  // remove useless edges.
  if(m_reduceFormula) removeSubsumEdges(hypergraph);
}// collectRelevantIdxClauses


/**
   We remove from the hypergraph the edges that are subsumed by another one.

   @param[out] hypergraph, the list of hyper edges.
 */
void PartitioningHeuristicBipartite::removeSubsumEdges(
    std::vector< std::vector<unsigned> > &hypergraph)
{
  unsigned i, j;
  for(i = j = 0 ; i<hypergraph.size() ; i++)
  {
    if(!hypergraph[i].size()) continue; // i is then removed.
    
    // mark the varaibles of the current edge.
    for(auto &x : hypergraph[i]) m_markedVar[x] = true;

    // visit the other edges to compute those that subsubmed or are subsubmed.
    bool subsumed = false;
    for(unsigned k = i + 1 ; k<hypergraph.size() ; k++)
    {
      unsigned cpt = 0;
      for(auto &x : hypergraph[k]) if(m_markedVar[x]) cpt++;

      if(cpt == hypergraph[k].size()) subsumed = true; // the current edge is smaller then include
      if(cpt == hypergraph[i].size()) hypergraph[k].clear(); // the edges k subsums i
    }
    
    for(auto &x : hypergraph[i]) m_markedVar[x] = false;     // reinit
    if(!subsumed){if(i != j) hypergraph[j++] = hypergraph[i]; else j++;} // copy or not ...
  }

  hypergraph.resize(j);
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
    std::vector< std::vector<unsigned> > &hypergraph,
    std::vector<int> &partition,
    std::vector<unsigned> &indices)
{
  bool clash = false;
  int part = 0;
  for(unsigned i = 0 ; i<hypergraph.size() ; i++)
  {
    clash = false;
    part = partition[hypergraph[i][0]];

    for(unsigned j = 1 ; !clash && j<hypergraph[i].size() ; j++) clash = part != partition[hypergraph[i][j]];
    if(clash) indices.push_back(i);
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
  std::vector< std::vector<unsigned> > hypergraph;
  constructHyperGraph(component, m_equivClass, hypergraph);
  m_pm->computePartition(hypergraph, m_partition);
  
  // collect the cut.
  extractCutFromHyperGraph(hypergraph, m_partition, cutSet);
  
  m_om.postUpdate(unitEquiv);
} // component
} // d4
