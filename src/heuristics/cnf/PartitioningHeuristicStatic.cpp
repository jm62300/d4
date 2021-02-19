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
#include "src/exceptions/FactoryException.hpp"
#include "PartitioningHeuristicStatic.hpp"
#include "PartitioningHeuristicStaticNone.hpp"
#include "PartitioningHeuristicStaticDual.hpp"
#include "PartitioningHeuristicStaticPrimal.hpp"

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

  m_pm = NULL;
  m_hypergraphExtractor = NULL;
} // constructor


/**
   Destructor.
*/
PartitioningHeuristicStatic::~PartitioningHeuristicStatic()
{
  if(m_hypergraphExtractor) delete m_hypergraphExtractor;
  if(m_pm) delete m_pm;
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
    std::vector<int> &partition,
    std::vector<unsigned> &mappingEdge,
    std::vector<Var> &mappingVar,
    std::vector<Strata> &stack,
    unsigned &level)
{  
  std::vector<unsigned> cutSet, indicesFirst, indicesSecond;
  splitWrtPartition(m_hypergraph, partition, mappingEdge,
                    cutSet, indicesFirst, indicesSecond);

  unsigned fatherId = stack.back().fatherId;
  unsigned currentId = (cutSet.size()) ? level : fatherId;
  stack.pop_back();  
  
  if(cutSet.size())
  {
    setCutSetBucketLevelFromEdges(hypergraph, partition, cutSet,
                                  mappingVar, level);
    assert(fatherId < m_separtorLevel.size());
    m_separtorLevel[fatherId] = level;
    level++;    
    m_separtorLevel.push_back(level);
  } else
  {
    // special case 1.
    if(!indicesFirst.size() && indicesSecond.size())
      assignLevel(hypergraph, currentId, indicesSecond, mappingVar, level);
    
    // special case 2.
    if(!indicesSecond.size() && indicesFirst.size())
      assignLevel(hypergraph, currentId, indicesFirst, mappingVar, level);
  }

  if(indicesSecond.size() > LIMIT) stack.push_back({currentId, indicesSecond});
  else assignLevel(hypergraph, currentId, indicesSecond, mappingVar, level);

  if(indicesFirst.size() > LIMIT) stack.push_back({currentId, indicesFirst});
  else assignLevel(hypergraph, currentId, indicesFirst, mappingVar, level);
} // distributePartition


/**
   Assign a set of mappingVar[indices] to their level.

   @param[in] hypergraph, the hyper graph that gives the edges.
   @param[in] idFather, the id of the node that created the sub tree.
   @param[in] indices, the set of edges (i in indices, then hypergraph[i] is an edge).
   @param[in] mappingVar, can be used to get the variable associated to an edge.
   @param[out] level, the current level.
 */
void PartitioningHeuristicStatic::assignLevel(
    std::vector<std::vector<unsigned> > &hypergraph,
    unsigned idFather,
    std::vector<unsigned> &indices,
    std::vector<Var> &mappingVar,
    unsigned &level)
{
  setBucketLevelFromEdges(hypergraph, indices, mappingVar, level);    
  if(indices.size())
  {
    m_separtorLevel[idFather] = level;
    level++;
    m_separtorLevel.push_back(level);
  }
} // assignLevel

/**
   Split the hyper graph into two parts that are induced by the given partition.

   @param[in] hypergraph, the hyper graph we search to split.
   @param[in] partition, a partition of the nets.
   @param[in] mappingEdge, to get the idx in the saved hyper graph.
   @param[out] cutset, the cutset, that is the edge 
   @param[out] indicesFirst, the set of edges regarding the first partition.
   @param[out] indicesSecond, the set of edges regarding the second partition.
*/
void PartitioningHeuristicStatic::splitWrtPartition(
    HyperGraph &hypergraph,
    std::vector<int> &partition,
    std::vector<unsigned> &mappingEdge,
    std::vector<unsigned> &cutSet,
    std::vector<unsigned> &indicesFirst,
    std::vector<unsigned> &indicesSecond)
{
  for(auto &edge : hypergraph)
  {    
    bool clash = false;
    int part = partition[edge[0]];
    for(unsigned i = 1 ; !clash && i<edge.getSize() ; i++)
      clash = part != partition[edge[i]];

    if(clash) cutSet.push_back(mappingEdge[edge.getId()]);
    else
    {
      if(part) indicesFirst.push_back(mappingEdge[edge.getId()]);
      else indicesSecond.push_back(mappingEdge[edge.getId()]);
    }
  }
} // splitWrtPartition


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
  std::vector<std::vector<unsigned>> savedHyperGraph;
  saveHyperGraph(savedHyperGraph);

  // preparation. 
  std::vector<int> partition(m_maxNbNodes, 0);
  std::vector<Var> indexToVar(m_maxNbEdges, 0);

  // init the stack with all the edges.
  std::vector<Strata> stack;
  Strata strata = {0, std::vector<unsigned>()};
  for(unsigned i = 0 ; i<savedHyperGraph.size() ; i++) strata.part.push_back(i);
  stack.push_back(strata);
  
  // reinit the bucket for all.
  m_separtorLevel.clear();
  m_separtorLevel.push_back(0);
  for(auto &b : m_bucketNumber) b = 0; 
  unsigned level = 1;
  
  // iteratively consider sub-graph.
  while(stack.size())
  {
    Strata &strata = stack.back();
    std::vector<unsigned> &current = strata.part;
    setHyperGraph(savedHyperGraph, current, m_hypergraph);
    
    m_pm->computePartition(m_hypergraph, Level::QUALITY, partition);

    // get the cut and split the current set of variables.
    distributePartition(savedHyperGraph, partition, current,
                        considered, stack, level);
  }
  
  // set the equivalence.
  for(auto v : component)
  {
    if(m_bucketNumber[v]) continue;

    if(v == equivClass[v]) m_bucketNumber[v] = level;
    else m_bucketNumber[v] = m_bucketNumber[equivClass[v]];
  }
} // computeDecomposition


/**
   Save the current hyper graph.

   @param[out] savedHyperGraph, the structure where is saved the graph.
*/
void PartitioningHeuristicStatic::saveHyperGraph(
    std::vector<std::vector<unsigned> > &savedHyperGraph)
{
  for(auto edge : m_hypergraph)
  {
    savedHyperGraph.push_back(std::vector<unsigned>());
    std::vector<unsigned> &tmp = savedHyperGraph.back();
    for(auto v : edge) tmp.push_back(v);
  }
} // savedHyperGraph


/**
   Set the hyper graph regarding the given set of variables and the saved
   hyper graph.

   @param[in] savedHyperGraph, the current hyper graph.
   @param[in] indices, the current set of edges' indices.
   @param[out] hypergraph, the computed hyper graph.
*/
void PartitioningHeuristicStatic::setHyperGraph(
    std::vector<std::vector<unsigned> > &savedHyperGraph,
    std::vector<unsigned> &indices,
    HyperGraph &hypergraph)
{
  unsigned *edges = hypergraph.getEdges();
  hypergraph.setSize(0);

  for(auto idxEdge : indices)
  {
    std::vector<unsigned> &tmp = savedHyperGraph[idxEdge];
    *edges = tmp.size();
    for(unsigned i = 0 ; i<tmp.size() ; i++) edges[i + 1] = tmp[i];      
    edges += *edges + 1;
    hypergraph.incSize();
  }
} // setHyperGraph


/**
   Generate a static partitioner regarding the given option list.

   @param[in] vm, the option list.
   @param[in] s, a wrapper to a solver.
   @param[in] om, a formula manager.
   @param[in] nbClause, the number of clauses of the formula.
   @param[in] nbVar, the number of variables of the formula.
   @param[in] sumSize, the number of literals of the formula.
   @param[in] type, a string that gives the partioner type.

   \return a static partioner.
 */
PartitioningHeuristicStatic *PartitioningHeuristicStatic::makePartitioningHeuristicStatic(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om,
    int nbClause,
    int nbVar,
    int sumSize,
    const std::string &type)
{
  std::string opt = vm["partitioning-heuristic-bipartite-phase"].as<std::string>();

  if(opt == "none")
    return new PartitioningHeuristicStaticNone(vm, s, om, nbClause, nbVar, sumSize);
  
  if(opt == "dual" || (opt == "natural" && type == "dual"))
    return new PartitioningHeuristicStaticDual(vm, s, om, nbClause, nbVar, sumSize);

  if(opt == "primal" || (opt == "natural" && type == "primal"))
    return new PartitioningHeuristicStaticPrimal(vm, s, om, nbClause, nbVar, sumSize);

  throw (FactoryException("Cannot create a PartitioningHeuristic",
                          __FILE__, __LINE__));
} // makePartitioningHeuristicStatic

} // d4
