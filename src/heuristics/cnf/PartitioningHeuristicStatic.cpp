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

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
*/
PartitioningHeuristicStatic::PartitioningHeuristicStatic(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om) :
    PartitioningHeuristicStatic(vm, s, om,
                                dynamic_cast<SpecManagerCnf&>(om).getNbClause(),
                                dynamic_cast<SpecManagerCnf&>(om).getNbVariable(),
                                dynamic_cast<SpecManagerCnf&>(om).getSumSizeClauses())
{  
} // constructor


/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
   @param[in] nbClause, the number of clauses.
   @param[in] nbVar, the number of variables.
   @param[in] sumSize, which give the number of literals.
*/
PartitioningHeuristicStatic::PartitioningHeuristicStatic(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om,
    int nbClause,
    int nbVar,
    int sumSize) : m_s(s), m_om(dynamic_cast<SpecManagerCnf&>(om))
{
  m_nbVar = nbVar;
  m_nbClause = nbClause;  
  
  m_em.initEquivExtractor(m_nbVar + 1);

  // get the options.
  m_reduceFormula = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  m_equivSimp = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();
  
  m_isInitialized = false;
  m_bucketNumber.resize(m_nbVar + 2, 0);
} // constructor


/**
   Destructor.
*/
PartitioningHeuristicStatic::~PartitioningHeuristicStatic()
{
  delete m_hypergraphExtractor;
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
  std::cout << "c [TREE DECOMPOSITION] Start tree decomposition generation ... " << std::flush;
  computeDecomposition(component, equivClass, equivVar, m_bucketNumber);
  std::cout << "done\n";
  
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

   @param[in] indicesFirst, the first parition.
   @param[in] indicesSecond, the second partition.
   @param[in] mappingVar, to get the variable associate with the index.
   @param[in] cutIsempty, specify if the partition that generates the two set of
   indices come from an empty cut set.   
   @param[out] stack, the current stack of set of variables (will receive
   indicesFirst and indicesSecond if their size is large enough).
   @param[out] level, the current level where are assigned the variables in
   their bucket.   
*/
void PartitioningHeuristicStatic::distributePartition(
    std::vector<std::vector<unsigned> > &hypergraph,
    std::vector<unsigned> &indicesFirst,
    std::vector<unsigned> &indicesSecond,
    std::vector<Var> &mappingVar,
    bool cutIsEmpty,
    std::vector< std::vector<unsigned> > &stack,
    unsigned &level)
{
  // special case 1.
  if(cutIsEmpty && !indicesFirst.size() && indicesSecond.size())
  {
    setBucketLevelFromEdges(hypergraph, indicesSecond, mappingVar, level);
    level++;
    return;
  }

  // special case 2.
  if(cutIsEmpty && !indicesSecond.size() && indicesFirst.size())
  {
    setBucketLevelFromEdges(hypergraph, indicesFirst, mappingVar, level);
    level++;
    return;
  }
  
  if(indicesFirst.size() > LIMIT) stack.push_back(indicesFirst);
  else
  {
    setBucketLevelFromEdges(hypergraph, indicesFirst, mappingVar, level);
    if(indicesFirst.size()) level++;
  }

  if(indicesSecond.size() > LIMIT) stack.push_back(indicesSecond);
  else
  {
    setBucketLevelFromEdges(hypergraph, indicesSecond, mappingVar, level);
    if(indicesSecond.size()) level++;
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
  using Level = PartitionerManager::Level;
  
  // construct the hypergraph
  std::vector<Var> considered;
  m_hypergraphExtractor->constructHyperGraph(
      m_om, component, equivClass, equivVar,
      m_reduceFormula, considered, m_hypergraph);

  // save the hyper graph.
  std::vector<std::vector<unsigned> > savedHyperGraph;
  saveHyperGraph(savedHyperGraph);

  // preparation. 
  std::vector<int> partition(m_maxNbNodes, 0);
  std::vector<Var> indexToVar(m_maxNbEdges, 0);

  // init the stack with all the edges.
  std::vector< std::vector<unsigned> > stack;
  stack.push_back(std::vector<unsigned>());
  for(unsigned i = 0 ; i<savedHyperGraph.size() ; i++) stack[0].push_back(i);

  // reinit the bucket for all.
  for(auto &b : m_bucketNumber) b = 0; 
  unsigned level = 1;
  
  // iteratively consider sub-graph.
  while(stack.size())
  {    
    std::vector<unsigned> &current = stack.back();
    setHyperGraph(savedHyperGraph, current, m_hypergraph);
    m_pm->computePartition(m_hypergraph, Level::QUALITY, partition);

    // get the cut and split the current set of variables.
    std::vector<Var> cutSet;
    std::vector<unsigned> indicesFirst, indicesSecond;
    
    m_hypergraphExtractor->splitWrtPartition(
        m_hypergraph, partition, considered, current,
        cutSet, indicesFirst, indicesSecond);

    // set the level for the current cut set.
    for(auto v : cutSet) m_bucketNumber[v] = level;
    if(cutSet.size()) level++;
    stack.pop_back();

    distributePartition(savedHyperGraph, indicesFirst, indicesSecond,
                        considered, !cutSet.size(), stack, level);
  }
  
  // set the equivalence.
  for(auto v : component)
  {
    if(m_bucketNumber[v]) continue;

    if(v == equivClass[v]) m_bucketNumber[v] = level;
    else m_bucketNumber[v] = m_bucketNumber[equivClass[v]];
  }
} // computeDecomposition

} // d4
