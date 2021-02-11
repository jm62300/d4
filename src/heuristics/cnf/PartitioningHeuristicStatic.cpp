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

#include "PartitioningHeuristicStatic.hpp"

namespace d4
{
/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
*/
PartitioningHeuristicStatic::PartitioningHeuristicStatic(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om) :
    PartitioningHeuristicStatic(vm, _s, _om,
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
PartitioningHeuristicStatic::PartitioningHeuristicStatic(
    po::variables_map &vm,
    WrapperSolver &_s,
    SpecManager &_om,
    int _nbClause,
    int _nbVar,
    int _sumSize) : m_s(_s), m_om(dynamic_cast<SpecManagerCnf&>(_om))
{
  m_nbVar = _nbVar;
  m_nbClause = _nbClause;  
  
  m_pm = PartitionerManager::makePartitioner(vm, m_nbClause, m_nbVar, _sumSize);
  m_hypergraph.init(m_nbVar + m_nbClause + _sumSize + 1);  
  m_hypergraphExtractor = new HyperGraphExtractorDual(m_nbVar, m_nbClause);
  
  m_em.initEquivExtractor(m_nbVar + 1);

  // get the options.
  m_reduceFormula = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  m_equivSimp = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();
  
  m_isInitialized = false;
  m_bucketNumber.resize(m_nbVar + 2, 0);
  m_mapVar.resize(m_nbVar + 2, 0);

  init();
} // constructor


/**
   Destructor.
*/
PartitioningHeuristicStatic::~PartitioningHeuristicStatic()
{
  delete m_pm;
} // destructor


/**
   Initialize the bucket level.
 */
void PartitioningHeuristicStatic::init()
{
  m_isInitialized = true;

  // the list of all variables.
  std::vector<Var> component;
  for(unsigned i = 1 ; i <= m_nbVar ; i++) component.push_back(i);

  // search for equiv class if requiered.
  std::vector<Lit> unitEquiv;
  std::vector<Var> equivClass;
  std::vector< std::vector<Var> > equivVar;  
  equivClass.resize(m_nbVar + 1, 0);
  
  if(m_equivSimp) PartitioningHeuristic::computeEquivClass(
         m_em, m_s, component, unitEquiv, equivClass, equivVar);
  else for(auto &v : component) equivClass[v] = v;
  
  // synchronize the SAT solver and the spec manager.
  m_om.preUpdate(unitEquiv);  

  // compute the decomposition.
  computeDecomposition(component, equivClass, equivVar, m_bucketNumber);
  
  // restore the initial state.
  m_om.postUpdate(unitEquiv);
}// init


/**
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables.
   @param[out] cutSet, the cut set we compute.
*/
void PartitioningHeuristicStatic::computeCutSet(
    std::vector<Var> &component,
    std::vector<Var> &cutSet)
{
  assert(m_isInitialized);
  
  // search for the next variable regarding the saved level.
  unsigned minLevel = m_nbVar;

  for(auto v : component)
  {
    if(m_bucketNumber[v] < minLevel)
    {
      cutSet.clear();
      minLevel = m_bucketNumber[v];
    }

    if(m_bucketNumber[v] == minLevel) cutSet.push_back(v);
  }

  assert(cutSet.size());
} // component


/**
   Split and assign variables.

   @param[in] savedHyperGraph, the current hyper graph.
   @param[in] current, the current set of variables.
   @param[in] partition, the computed partition of clauses (dual graph).
   @param[out] cutSet, the extracted cut set.
   @param[out] setLeft, the first parition.
   @param[out] setRight, the second partition.   
   @param[out] stack, the current stack of set of variables (will receive
   setLeft and setRight if their size is large enough).   
   @param[out] level, the current level where are assigned the variables in
   their bucket.
*/
void PartitioningHeuristicStatic::splitVarWrtPartition(
    std::vector<std::vector<unsigned> > &savedHyperGraph,
    std::vector<Var> &current,
    std::vector<int> &partition,
    std::vector<Var> &cutSet,
    std::vector<Var> &setLeft,
    std::vector<Var> &setRight,
    std::vector< std::vector<Var> > &stack,
    unsigned &level)
{
  for(auto v : current)
  {
    unsigned idxEdge = m_mapVar[v];
    std::vector<unsigned> &tmp = savedHyperGraph[idxEdge];

    bool clash = false;
    int part = partition[tmp[0]];
    for(unsigned i = 1 ; !clash && i<tmp.size() ; i++)
      clash = part != partition[tmp[i]];

    if(clash) cutSet.push_back(v);
    else { if(part) setLeft.push_back(v); else setRight.push_back(v); }
  }
    
  // set the level for the current cut set.
  for(auto v : cutSet)
  {
    assert((unsigned) v <= m_nbVar);
    m_bucketNumber[v] = level;
  }

  // pop the current set of variables and add the two partitioned set.    
  if(cutSet.size()) level++;    
  stack.pop_back();

  if(setLeft.size() > LIMIT) stack.push_back(setLeft);
  else
  {
    for(auto v : setLeft) m_bucketNumber[v] = level;
    if(setLeft.size()) level++;
  }

  if(setRight.size() > LIMIT) stack.push_back(setRight);
  else
  {
    for(auto v : setRight) m_bucketNumber[v] = level;
    if(setRight.size()) level++;
  }
} // splitVarWrtPartition


/**
   Search a decomposition tree regarding a component.

   @param[in] component, the set of varaibles the problem is constructed on.
   @param[out] bucketNumber, the decomposition tree in term of index.
 */
void PartitioningHeuristicStatic::computeDecomposition(
    std::vector<Var> &component,
    std::vector<Var> &equivClass,
    std::vector< std::vector<Var> > &equivVar,
    std::vector<unsigned> &bucketNumber)
{
  // construct the hypergraph
  std::vector<Var> considered;
  m_hypergraphExtractor->constructHyperGraph(
      m_om, component, equivClass, equivVar,
      m_reduceFormula, considered, m_hypergraph);

  // save the hyper graph.
  std::vector<std::vector<unsigned> > savedHyperGraph;
  saveHyperGraph(savedHyperGraph);

  // keep the variables' information.
  for(unsigned i = 0 ; i<considered.size() ; i++) m_mapVar[considered[i]] = i;

  // preparation.
  std::vector<int> partition;
  std::vector< std::vector<Var> > stack;
  partition.resize(m_nbClause + 1, 0);
  stack.push_back(considered);

  for(auto &b : m_bucketNumber) b = 0; // reinit
  unsigned level = 1;
  
  // iteratively consider sub-graph.
  while(stack.size())
  {
    std::vector<Var> &current = stack.back();
    setHyperGraph(savedHyperGraph, current, m_mapVar, m_hypergraph);
    m_pm->computePartition(m_hypergraph, partition);

    // get the cut and split the current set of variables.
    std::vector<Var> cutSet;
    std::vector<Var> setLeft, setRight;
    splitVarWrtPartition(savedHyperGraph, current, partition, cutSet,
                         setLeft, setRight, stack, level);
  }
  
  // set the equivalence.
  for(auto v : component)
  {
    if(m_bucketNumber[v]) continue;
    else
    {
      m_bucketNumber[v] = (v == equivClass[v]) ?
                          level : m_bucketNumber[equivClass[v]];
    }
  }
} // computeDecomposition

} // d4
