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

  // initialize the vector.
  m_mapVar.resize(m_nbVar + 1, 0);
  m_markedVar.resize(m_nbVar + 1, false);
  m_equivClass.resize(m_nbVar + 1, 0);
  m_useLessVariable.resize(m_nbVar + 1, false);
  m_partition.resize(m_nbClause + 1, 0);
  m_markedClauses.resize(m_nbClause + 1, false);
  
  // get the options.
  m_reduceFormula = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  m_equivSimp = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();

  m_hypergraphCapacity = m_nbVar + m_nbClause + _sumSize + 1;
  m_hypergraph = new unsigned[m_hypergraphCapacity];
  m_hypergraphSize = 0;
} // constructor


/**
   Destructor.
*/
PartitioningHeuristicBipartiteDual::~PartitioningHeuristicBipartiteDual()
{
  delete[] m_hypergraph;
  delete m_pm;
} // destructor


/**
   Print out the hyper graph.

   @param[in] hypergraph, the hypergraph data in row.
   @param[in] size, the number of elements in hypergraph.
*/
void PartitioningHeuristicBipartiteDual::displayHyperGraph(
    unsigned *hypergraph,
    unsigned size)
{
  unsigned *p = hypergraph;
  for(unsigned i = 0 ; i<size ; i++)
  {
    for(unsigned j = 0 ; j<*p ; j++) std::cout << p[1 + j] << " ";
    std::cout << "\n";
    p += *p + 1;
  }
} // displayHyperGraph

/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).
   We try to minimize the cut in a greedy fashion.

   @param[in] considered, the label variables for the edges.
   @param[in] partition, the array that gives the partition.
   @param[out] cutSet, the computed cutset.
*/
void PartitioningHeuristicBipartiteDual::extractCutFromHyperGraph(
    std::vector<Var> &considered,
    std::vector<int> &partition,
    std::vector<int> &cutSet)
{
  std::vector<unsigned> indices;
  clashHyperEdgeIndex(partition, indices);
  for(auto &i : indices) cutSet.push_back(considered[i]);
} // extractCutFromClauses


/**
   Get the clauses we will use in the partitioning algorithm.

   @param[in] component, the set of variables we focus on.
   @param[in] equiClass, the equivalence class.   
   @param[out] considered, make the correspondance between the hyperedges and
   the variables.
*/
void PartitioningHeuristicBipartiteDual::constructHyperGraph(
    std::vector<Var> &component,
    std::vector<Var> &equivClass,
    std::vector<Var> &considered)
{
  unsigned pos = 0;
  m_hypergraphSize = 0;  
  for(auto &v : component)
  {
    unsigned &size = m_hypergraph[pos++];
    size = 0;

    for(auto &idx : m_om.getVecIdxClause(Lit::makeLitFalse(v)))
    {
      m_hypergraph[pos++] = idx;
      size++;
    }

    for(auto &idx : m_om.getVecIdxClause(Lit::makeLitTrue(v)))
    {
      m_hypergraph[pos++] = idx;
      size++;
    }
    
    m_hypergraphSize++;
    considered.push_back(v);
  }

  // displayHyperGraph(m_hypergraph, m_hypergraphSize);

  // remove useless edges.
  // if(m_reduceFormula) removeSubsumEdges(m_hypergraph, m_hypergraphSize);
}// collectRelevantIdxClauses

/**
   Associate for each variable in the component an equivalence class.

   @param[in] component, the set of variables of the component we want to cut.
   @param[out] unitEquiv, the set of unit literals we find out.
   @param[out] equiClass, the equivalence class we computed (we suppose that the
   verctor is large enough and then we do not allocate).
*/
void PartitioningHeuristicBipartiteDual::computeEquivClass(
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
void PartitioningHeuristicBipartiteDual::clashHyperEdgeIndex(
    std::vector<int> &partition,
    std::vector<unsigned> &indices)
{
  bool clash = false;
  int part = 0;
  unsigned *edge = m_hypergraph;  
  for(unsigned i = 0 ; i<m_hypergraphSize ; i++)
  {
    clash = false;
    part = partition[edge[1]];

    for(unsigned j = 1 ; !clash && j<*edge ; j++) clash = part != partition[edge[1 + j]];
    if(clash) indices.push_back(i);

    edge = &(edge[*edge + 1]); // next clause.
  }
} // clashHyperEdgeIndex

/**
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables.
   @param[out] cutSet, the cut set we compute.
*/
void PartitioningHeuristicBipartiteDual::computeCutSet(
    std::vector<Var> &component,
    std::vector<Var> &cutSet)
{
  // search for equiv class if requiered.
  std::vector<Lit> unitEquiv;
  computeEquivClass(component, unitEquiv, m_equivClass);
  
  // synchronize the SAT solver and the spec manager.
  m_om.preUpdate(unitEquiv);

  // construct the hypergraph
  std::vector<Var> considered;
  constructHyperGraph(component, m_equivClass, considered);
  m_pm->computePartition(m_hypergraph, m_hypergraphSize,
                         (std::function<bool(int)>) [](int i) {return true;},
                         m_partition);
  
  // collect the cut.
  extractCutFromHyperGraph(considered, m_partition, cutSet);
  
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
