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
    WrapperSolver &_s,
    SpecManager &_om,
    unsigned options) : s(_s), om(dynamic_cast<SpecManagerCnf&>(_om))
{ 
  em.initEquivExtractor(om.getNbVariable());
  sumSize = om.getSumSizeClauses();

  // initialize the vector.
  inCurrentComponent.resize(om.getNbVariable(), false);
  mapVar.resize(om.getNbVariable(), 0);
  markedVar.resize(om.getNbVariable(), false);
  useLessVariable.resize(om.getNbVariable(), false);
  markedClauses.resize(om.getNbClause(), false);
  weightClause.resize(om.getNbClause(), 1);
  
  // allocate the memory
  pins = new int[sumSize];
  partweights = new int[2];
  xpins = new int[(om.getNbVariable() + 2)];
  vwghts = new int[(om.getNbVariable() + 2)];
  partvec = new int[(om.getNbClause() + 2)];
  cwghts = new int[(om.getNbClause() + 2)];

  // set all weightclause to 1
  for(int i = 0 ; i<(om.getNbClause() + 1) ; i++) cwghts[i] = 1;

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
   Compute a cutset by computing a bipartition of the hypergraph of the clauses.

   @param[in] component, the set of variables of the component we want to cut.
   @param[out] cutSet, the cut set we compute.
 */
void PartitioningHeuristicBipartite::computePartition(std::vector<Var> &component,
                                                      std::vector<Var> &cutSet)
{
  
} // component

} // d4
