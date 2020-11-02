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

#include "EquivExtractor.hpp"

namespace d4
{

/**
   Constructor.
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
EquivExtractor::EquivExtractor(int nbVar)
{
  initEquivExtractor(nbVar);
} // constructor


/**
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.   
 */
void EquivExtractor::initEquivExtractor(int nbVar)
{
  markedVar.resize(nbVar, false);
  markedVarInter.resize(nbVar, false);  
} // initEquivExtractor


/**
   Compute the of variable that are "propagated" whatever the phase of the given
   literal l.

   @param[in] s, a wrapper to a solver.
   @param[in] l, the literal we search for the "unit variables".
   @param[out] listVarPU, the resulting variables.

   \return false if assign l or ~l produces a conflict, true otherwise.
 */
bool EquivExtractor::interCollectUnit(WrapperSolver &s,
                                      Var v,
                                      std::vector<Var> &listVarPU)
{
  std::vector<Lit> listVarPosLit, listVarNegLit;
  if(!s.decideAndComputeUnit(Lit(v, false), listVarPosLit)) return false;
  if(!s.decideAndComputeUnit(Lit(v, true), listVarNegLit)) return false;

  // intersection.
  for(auto &l : listVarPosLit) markedVarInter[l.var()] = true;
  for(auto &l : listVarNegLit)
    if(markedVarInter[l.var()]) listVarPU.push_back(l.var());
  for(auto &l : listVarPosLit) markedVarInter[l.var()] = false;

  return true;
} // interCollectUnit


/**
   Research equivalences in the set of variable v.

   @param[in] s, a wrapper to a solver.
   @param[in] v, the set of variables we search in.
   @param[out] equivVar, le resulting equivalences.
 */
void EquivExtractor::searchEquiv(WrapperSolver &s,
                                 std::vector<Var> &vars,
                                 std::vector< std::vector<Var> > &equivVar)
{
  std::vector<Var> reinit;
      
  for(auto &v : vars)
  {
    if(markedVar[v] || s.varIsAssigned(v)) continue;
    
    std::vector<Var> eqv;
    if(interCollectUnit(s, v, eqv))
    {      
      if(eqv.size() == 1) continue;
      equivVar.push_back(eqv);
      for(auto &vv : eqv)
      {
        markedVar[vv] = true;
        reinit.push_back(vv);
      }      
    } 
  }
  
  for(auto &v : reinit) markedVar[v] = false;
} // searchEquiv

} // d4
