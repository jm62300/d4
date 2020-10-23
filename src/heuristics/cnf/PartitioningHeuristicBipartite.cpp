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
#include <3rdParty/patoh/patoh.h>

namespace d4
{

/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
 */
PartitioningHeuristicBipartite::PartitioningHeuristicBipartite(
    WrapperSolver &_s,
    SpecManager &_om,
    unsigned options) :
    PartitioningHeuristicBipartite(_s, _om, options,
                             dynamic_cast<SpecManagerCnf&>(_om).getNbVariable(),
                             dynamic_cast<SpecManagerCnf&>(_om).getNbClause(),
                             dynamic_cast<SpecManagerCnf&>(_om).getSumSizeClauses())
{ 
} // constructor


/**
   Constructor.

   @param[in] _s, a wrapper on a solver.
   @param[in] _om, a structure manager.
 */
PartitioningHeuristicBipartite::PartitioningHeuristicBipartite(
    WrapperSolver &_s,
    SpecManager &_om,
    unsigned options,
    int _nbClause,
    int _nbVar,
    int _sumSize) : s(_s), om(dynamic_cast<SpecManagerCnf&>(_om))
{ 
  em.initEquivExtractor(_nbVar);
  sumSize = _sumSize;
  nbVar = _nbVar;
  nbClause = _nbClause;

  // initialize the vector.
  inCurrentComponent.resize(nbVar, false);
  mapVar.resize(nbVar, 0);
  markedVar.resize(nbVar, false);
  useLessVariable.resize(nbVar, false);
  markedClauses.resize(nbClause, false);
  weightClause.resize(nbClause, 1);
  
  // allocate the memory
  pins = new int[sumSize];
  partweights = new int[2];
  xpins = new int[(nbVar + 2)];
  vwghts = new int[(nbVar + 2)];
  partvec = new int[(nbClause + 2)];
  cwghts = new int[(nbClause + 2)];

  // set all weightclause to 1
  for(int i = 0 ; i<(nbClause + 1) ; i++) cwghts[i] = 1;

  // get the options.
  reduceFormula = options & 1;
  equivSimp = (options>>1) & 1;  
} // constructor


/**
   Destructor.
 */
PartitioningHeuristicBipartite::~PartitioningHeuristicBipartite()
{
  delete[] pins;
  delete[] partweights;
  delete[] xpins;
  delete[] vwghts;
  delete[] partvec;
  delete[] cwghts;
} // destructor


/**
   Check all the hyper edges in order to extract those their are conflictual
   (i.e. there are belong to at least two components).

   @param[in] hyperEdges, the list of hyper edges.
   @param[in] pv, the array that gives the partition.

   \return the indices of the hyper edges that are between several components.
 */
void PartitioningHeuristicBipartite::extractCutFromClauses(
    std::vector< std::vector<int> > &hyperEdges,
    std::vector<int> &cutSet,
    int *pv)
{
  for(unsigned i = 0 ; i<hyperEdges.size() ; i++)
  {
    std::vector<int> &c = hyperEdges[i];
    bool split = false;
    
    for(unsigned j = 1 ; !split && j<c.size() ; j++)
      split = pv[c[j]] != pv[c[0]];

    if(split) cutSet.push_back(i);
  }
} // extractCutFromClauses


/**
   Construct the occurrence map from the set of clauses.

   @param[out] hyperEdges, the list of hyper edges. We suppose that hyperEdges
   is nig enough, that means its size is at least the number of variables of the
   current component.   
   @param[in] idxClauses, the set of index of clauses.
 */
void PartitioningHeuristicBipartite::buildOccMap(
    std::vector< std::vector<int> > &hyperEdges,
    std::vector<int> &idxCl)
{
  for(auto &v : hyperEdges) v.clear();
  for(unsigned i = 0 ; i<idxCl.size() ; i++)
  {
    int id = idxCl[i];
    for(auto &l : om.getClause(id))
    {
      if(!om.litIsAssigned(l) && !useLessVariable[l.var()])
        hyperEdges[mapVar[l.var()]].push_back(i);
    }
  }
} // buildOccMap

/**
   Remove useless variables and ajust the hypergraph, mapVar and save the new
   set of variables.

   @param[in] component, the initial set of variables
   @param[out] occMap, the occurrence map
   @param[out] useFulVariable, the set of kept variables
 */
void PartitioningHeuristicBipartite::clearSetOfVariable(
    std::vector<Var> &component,
    std::vector< std::vector<int> > &hypergraph,
    std::vector<Var> &useFulVariable)
{
  hypergraph.clear();
  for(auto &v : component)
    if(!om.varIsAssigned(v) && !useLessVariable[v])
    {
      mapVar[v] = useFulVariable.size();
      useFulVariable.push_back(v);
      hypergraph.push_back(std::vector<int>());
    }
} // clearSetOfVariable


/**
   Search if some variable can be drop from the graph.

   @param[in] component, the set of variables
   @param[in] hypergraph, occurrence lists
   @param[in] idxClauses, correspondance between the occurrence lists and the
   clauses
*/
void PartitioningHeuristicBipartite::computeUselessVariables(
    std::vector<Var> &component,
    std::vector< std::vector<int> > &hypergraph,
    std::vector<int> &idxClauses)
{
  for(unsigned i = 0 ; i<component.size() ; i++)
  {
    if(!hypergraph[i].size()) continue;

    // init with the first clause
    std::vector<Lit> &c = om.getClause(idxClauses[hypergraph[i][0]]);
    for(auto &l : c) if(!om.litIsAssigned(l)) markedVar[l.var()] = true;
    
    bool notUseFul = true;
    for(unsigned j = 1 ; notUseFul && j<hypergraph[i].size() ; j++)
    {
      std::vector<Lit> &d = om.getClause(idxClauses[hypergraph[i][j]]);
      for(unsigned k = 0 ; notUseFul && k<d.size() ; k++)
        if(!om.litIsAssigned(d[k])) notUseFul = markedVar[d[k].var()]; 
    }
    if(notUseFul) useLessVariable[component[i]] = true;

    // reinit the variable we marked.
    for(auto &l : c) markedVar[l.var()] = false;
  } 
} // computeUselessVariables


/**
   Remove useless clauses (a clause is useless if I can find
   another one with exactly the same variable).

   @param[out] idxClauses, the set of index of clauses we want to purge
*/
void PartitioningHeuristicBipartite::computeUselessClauses(
    std::vector<int> &idxClauses,
    std::vector< std::vector<int> > &hypergraph)
{
  for(unsigned i = 0 ; i<idxClauses.size() ; i++)
  {
    if(idxClauses[i] == -1) continue;
    std::vector<Lit> &c = om.getClause(idxClauses[i]);    
    for(auto &l : c) markedVar[l.var()] = true; // we mark the clause

    // search a variable in c that minimized the number of occurrences.
    // we also compute the number of available variables.
    Var v = var_Undef;
    int nbAvailableVar = 0;
    for(auto &l : c)
    {
      Var vp = l.var();
      if(!om.varIsAssigned(vp) && !useLessVariable[vp])
      {
        nbAvailableVar++;
        if(v == var_Undef ||
           hypergraph[mapVar[v]].size() > hypergraph[mapVar[vp]].size()) v = vp;
      }
    }

    if(v != var_Undef)
    {
      for(unsigned j = 0 ; j<hypergraph[mapVar[v]].size() ; j++)
      {
        unsigned idxCl = hypergraph[mapVar[v]][j];
        if(idxCl == i || idxClauses[idxCl] == -1) continue;
        std::vector<Lit> &d = om.getClause(idxClauses[idxCl]);

        int cpt = 0;
        for(unsigned k = 0 ; cpt < nbAvailableVar && k<d.size() ; k++)
          if(markedVar[d[k].var()] &&
             !om.litIsAssigned(d[k]) && !useLessVariable[d[k].var()]) cpt++;

        if(cpt >= nbAvailableVar) // we keep the largest clause
        {
          weightClause[idxClauses[idxCl]] += weightClause[idxClauses[i]];
          idxClauses[i] = -1;
          break;
        }
      }
    }
    
    for(auto &l : c) markedVar[l.var()] = false; // unmark
  }
} // computeUselessClauses


/**
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables of the component we want to cut.
   @param[out] cutSet, the cut set we compute.
 */
void PartitioningHeuristicBipartite::computePartition(
    std::vector<Var> &component,
    std::vector<Var> &cutVar)
{
  int cut;
  cutVar.clear();

  std::vector< std::vector<int> > hypergraph;
  std::vector<Var> notPlaced;
  for(unsigned i = 0 ; i<component.size() ; i++)
  {
    mapVar[component[i]] = i;
    inCurrentComponent[component[i]] = true;
    hypergraph.push_back(std::vector<int>());
  }


  std::vector<Lit> unitEquiv;
  std::vector< std::vector<Var> > equivVar;
  if(equivSimp)
  {
    em.searchEquiv(s, component, equivVar);
    s.whichAreUnits(component, unitEquiv);
  }
  om.preUpdate(unitEquiv);


  // collect the set of clauses
  om.getCurrentClauses(idxClauses, inCurrentComponent);
  for(auto &idx : idxClauses) weightClause[idx] = 1;
  buildOccMap(hypergraph, idxClauses);

  std::vector<Var> vUse;
  if(!reduceFormula) component = vUse;
  else
  {
    computeUselessVariables(component, hypergraph, idxClauses);
    computeUselessClauses(idxClauses, hypergraph);
    clearSetOfVariable(component, hypergraph, vUse);

    unsigned i, j;
    for(i = j = 0 ; i<idxClauses.size() ; i++)
      if(idxClauses[i] != -1) idxClauses[j++] = idxClauses[i];

    idxClauses.resize(i);
    buildOccMap(hypergraph, idxClauses);
  }

  
  if(equivSimp)
  {
    for(auto &classEquiv : equivVar)
    {
      assert(classEquiv.size());
      Var v = classEquiv.back();
      if(!inCurrentComponent[v] || useLessVariable[v]) continue;
      for(auto &idx : hypergraph[mapVar[v]]) markedClauses[idx] = true;
      
      for(auto &vv : classEquiv)
      {
        if(vv == v) continue;
        if(!inCurrentComponent[vv] || useLessVariable[vv]) continue;

        // transfer the clause in v
        for(auto &idx : hypergraph[mapVar[vv]])
        {
          if(!markedClauses[idx])
          {
            hypergraph[mapVar[v]].push_back(idx);
            markedClauses[idx] = true;
          }
        }
        
        useLessVariable[vv] = true;
      }

      for(auto &idx : hypergraph[mapVar[v]]) markedClauses[idx] = false;
    }

    // remove useless variables
    unsigned i, j;
    for(i = j = 0 ; i<vUse.size() ; i++)
      if(!useLessVariable[vUse[i]])
      {
        if(i != j) hypergraph[j] = hypergraph[i];
        mapVar[j] = vUse[i];
        vUse[j++] = vUse[i];
      }
    
    vUse.resize(i);
    hypergraph.resize(i);
  }
  

  // graph initialization
  int posPins = 0;
  for(unsigned i = 0 ; i<vUse.size() ; i++)
  {
    xpins[i] = posPins;
    for(auto &idx : hypergraph[i]) pins[posPins++] = idx;
  }
  xpins[vUse.size()] = posPins;

  // hypergraph partitioner
  PaToH_Parameters args;
  if(vUse.size() < 200)
    PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_DEFAULT);
  else PaToH_Initialize_Parameters(&args, PATOH_CONPART, PATOH_SUGPARAM_QUALITY);

  args._k = 2;
  args.seed = 1;

  for(unsigned i = 0 ; i<idxClauses.size() ; i++)
    cwghts[i] = weightClause[idxClauses[i]];
  
  PaToH_Alloc(&args, idxClauses.size(), vUse.size(), 1, cwghts, NULL, xpins, pins);
  PaToH_Part(&args, idxClauses.size(), vUse.size(), 1, 0, cwghts, NULL, xpins, pins,
             NULL, partvec, partweights, &cut);

  std::vector<int> idxEdges;
  extractCutFromClauses(hypergraph, idxEdges, partvec);
  for(auto &id : idxEdges) cutVar.push_back(vUse[id]);
  
  for(unsigned i = 0 ; i<component.size() ; i++)
    useLessVariable[component[i]] = inCurrentComponent[component[i]] = false;
  PaToH_Free();

  for(unsigned i = 0 ; i<equivVar.size() ; i++)
  {
    Var v = equivVar[i].back();
    bool isInCut = false;
    for(unsigned j = 0 ; j<cutVar.size() && !isInCut ; j++)
      isInCut = v == cutVar[j];
    if(!isInCut) continue;
    for(unsigned j = 0 ; j<equivVar[i].size() - 1 ; j++)
      if(inCurrentComponent[equivVar[i][j]]) cutVar.push_back(equivVar[i][j]);
  }

  om.postUpdate(unitEquiv);
} // component

} // d4
