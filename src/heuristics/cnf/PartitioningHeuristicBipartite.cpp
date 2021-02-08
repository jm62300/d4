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

   @param[in] vm, the option list.
   @param[in] om, the occurrence manager should be dedicated to CNF.
   @param[in] s, a SAT solver.
   @pararm[in] _nbClause, the number of clauses.
   @param[in] _nbVar, the number of variables.
   @param[in] _sumsize, the maximum size for the hyper graph.
 */
PartitioningHeuristicBipartite::PartitioningHeuristicBipartite(
    po::variables_map &vm,
    SpecManager &om,
    WrapperSolver &s,
    int _nbClause,
    int _nbVar,
    int _sumSize) :
    m_om(dynamic_cast<SpecManagerCnf&>(om)),
    m_s(s)
{ 
  m_nbVar = _nbVar;
  m_nbClause = _nbClause;
  
  m_em.initEquivExtractor(_nbVar + 1);

  // initialize the vectors.
  m_markedVar.resize(m_nbVar + 1, false);
  m_equivClass.resize(m_nbVar + 1, 0);
  
  // get the options.
  m_reduceFormula = vm["partitioning-heuristic-simplification-hyperedge"].as<bool>();
  m_equivSimp = vm["partitioning-heuristic-simplification-equivalence"].as<bool>();

  m_pm = NULL;
  m_hypergraphExtractor = NULL;
} // constructor


/**
   Destructor.
*/
PartitioningHeuristicBipartite::~PartitioningHeuristicBipartite()
{
  if(m_hypergraphExtractor) delete m_hypergraphExtractor;
  if(m_pm) delete m_pm;
} // destructor


/**
   Compute the equivalence class.

   @param[in] component, the set of variables.
   @param[out] unitEquiv, the set of literals that have been proved unit.   
   @param[out] equivClass, give the equivalent variable for each of them (the
   array should be large enough - size greater than the number of variables - to
   store the result).   
   @param[out] equivVar, equivalence class found.
 */
void PartitioningHeuristicBipartite::computeEquivClass(
      std::vector<Var> &component,
      std::vector<Lit> &unitEquiv,
      std::vector<Var> &equivClass,
      std::vector< std::vector<Var> > &equivVar)
{
  if(m_equivSimp) PartitioningHeuristic::computeEquivClass(
         m_em, m_s, component, unitEquiv, equivClass, equivVar);
  else for(auto &v : component) equivClass[v] = v;
} // computeEquivclass


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
  std::vector< std::vector<Var> > equivVar;
  computeEquivClass(component, unitEquiv, m_equivClass, equivVar);
  
  // synchronize the SAT solver and the spec manager.
  m_om.preUpdate(unitEquiv);  

  // construct the hypergraph
  std::vector<Var> considered;
  m_hypergraphExtractor->constructHyperGraph(
      m_om, component, m_equivClass, equivVar,
      m_reduceFormula, considered, m_hypergraph);
  
  m_pm->computePartition(m_hypergraph, m_partition);  

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
} // computeCutset

} // d4
