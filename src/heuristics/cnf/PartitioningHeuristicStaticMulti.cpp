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
#include "PartitioningHeuristicStaticMulti.hpp"

namespace d4
{
/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
*/
PartitioningHeuristicStaticMulti::PartitioningHeuristicStaticMulti(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om) : PartitioningHeuristicStaticMulti(
        vm, s, om,
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
PartitioningHeuristicStaticMulti::PartitioningHeuristicStaticMulti(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om,                                 
    int nbClause,
    int nbVar,
    int sumSize) :
    PartitioningHeuristicStatic(vm, s, om, nbClause, nbVar, sumSize)
{
  m_partitionStaticDual = new PartitioningHeuristicStaticSingleDual(
      vm, s, om, nbClause, nbVar, sumSize);

  m_partitionStaticPrimal = new PartitioningHeuristicStaticSinglePrimal(
      vm, s, om, nbClause, nbVar, sumSize);

  m_isStillOKDual = true;
  m_isStillOKPrimal = true;
} // constructor


/**
   Destructor.
 */
PartitioningHeuristicStaticMulti::~PartitioningHeuristicStaticMulti()
{
  if(m_partitionStaticDual) delete m_partitionStaticDual;
  if(m_partitionStaticPrimal) delete m_partitionStaticPrimal;
} // destructor


/**
   Initialize the bucket level.
*/
void PartitioningHeuristicStaticMulti::init()
{
  m_isInitialized = true;

  // the list of all variables.
  std::vector<Var> component;
  for(unsigned i = 1 ; i <= m_nbVar ; i++) component.push_back(i);

  // search for equiv class if requiered.
  std::vector<Lit> unitEquiv;  
  std::vector< std::vector<Var> > equivVar;  
  m_equivClass.resize(m_nbVar + 1, 0);
  
  if(m_equivSimp) PartitioningHeuristic::computeEquivClass(
         m_em, m_s, component, unitEquiv, m_equivClass, equivVar);
  else for(auto &v : component) m_equivClass[v] = v;
  
  // synchronize the SAT solver and the spec manager.
  m_om.preUpdate(unitEquiv);

  SpecManagerCnf *om = static_cast<SpecManagerCnf *>(&m_om);
  std::vector<bool> markedVar;
  markedVar.resize(m_nbVar + 1, false);
  for(unsigned i = 0 ; i<om->getNbClause() ; i++)
  {
    if(om->isSatisfiedClause(i) || om->getInitSize(i) < 10) continue;

    std::vector<Lit> &cl = om->getClause(i);
    for(auto &l : cl)
    {
      if(om->litIsAssigned(l)) continue;
      markedVar[l.var()] = true;
    }
  }

  unsigned cptMarked = 0, cpt = 0;
  for(auto &v : component)
  {
    if(om->varIsAssigned(v)) continue;
    if(markedVar[v]) cptMarked++;
    cpt++;
  }

  m_ratio = (double) cptMarked / (double) cpt;
  std::cout << "c [TREE DECOMPOSITION] cover ratio: " << m_ratio << "\n";

  // compute the decomposition.
  std::cout << "c [TREE DECOMPOSITION] Start tree decomposition generation ... " << std::flush;
  computeDecomposition(component, m_equivClass, equivVar);
  std::cout << "done\n";
  
  // restore the initial state.
  m_om.postUpdate(unitEquiv);
}// init



/**
   Ask if the current decomposition is still correct.

   @param[in] component, the set of variables.

   \return true if the tree decomposition is 'correct'.
 */
bool PartitioningHeuristicStaticMulti::isStillOk(
    std::vector<Var> &component)
{
  m_isStillOKDual = m_partitionStaticDual->isStillOk(component);
  m_isStillOKPrimal = m_partitionStaticPrimal->isStillOk(component);
  return m_isStillOKDual || m_isStillOKPrimal;
} // isStillOk



/**
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables.
   @param[out] cutSet, the cut set we compute.
*/
void PartitioningHeuristicStaticMulti::computeCutSet(
    std::vector<Var> &component,
    std::vector<Var> &cutSet)
{
  if(component.size() <= 10){cutSet = component; return;}

  if(m_ratio < 0.5) m_partitionStaticDual->computeCutSet(component, cutSet);
  else m_partitionStaticPrimal->computeCutSet(component, cutSet);  
} // component


/**
   Search a decomposition tree regarding a component.

   @param[in] component, the set of varaibles the problem is constructed on.
   @param[in] equivClass, the equivalence class for each variable.
   @param[in] equivVar, the list of equivalences.
   @param[out] bucketNumber, the decomposition tree in term of index.
*/
void PartitioningHeuristicStaticMulti::computeDecomposition(
    std::vector<Var> &component,
    std::vector<Var> &equivClass,
    std::vector< std::vector<Var> > &equivVar)
{
  m_isInitialized = true;
  m_partitionStaticDual->setIsInitialized(true);  
  m_partitionStaticDual->computeDecomposition(
      component, equivClass, equivVar,
      m_partitionStaticDual->getBucketNumber());
  
  m_partitionStaticPrimal->setIsInitialized(true);
  m_partitionStaticPrimal->computeDecomposition(
      component, equivClass, equivVar,
      m_partitionStaticPrimal->getBucketNumber());
} // computeDecomposition

} // d4
