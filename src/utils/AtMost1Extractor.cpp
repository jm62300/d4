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
#include <algorithm>

#include "AtMost1Extractor.hpp"

namespace d4
{

/**
   Constructor.
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.
 */
AtMost1Extractor::AtMost1Extractor(int nbVar)
{
  init(nbVar);
} // constructor


/**
   Init the structure with the good number of variables.

   @param[in] nbVar, the number of variables.   
 */
void AtMost1Extractor::init(int nbVar)
{
  m_nbVar = nbVar;
  m_markedVar.resize(nbVar + 1, false);
  m_stamp.resize(nbVar + 1, 0);
} // initAtMost1Extractor


/**
   Compute for each variable given in parameter the list of variables their are
   link by unit propagation. I.e. varBlock[v] = {a, b, ....} s.t. v -> a ...

   @param[in] s, the SAT solver.
   @param[in] vars, the set of variables we are looking for.
   @param[out] varBlock, the computed set of linked variables.
 */
void AtMost1Extractor::extractVarBlock(
    WrapperSolver &s,
    std::vector<Var> &vars,
    std::vector< std::vector<Var> > &varBlock)
{
  varBlock.clear();
  varBlock.resize(m_nbVar + 1, std::vector<Var>());
  
  for(auto v : vars)
  {
    Lit l = Lit::makeLit(v, false);
    std::vector<Lit> listPU;    
    s.decideAndComputeUnit(l, listPU);

    for(auto &x : listPU)
    {
      m_markedVar[x.var()] = true;
      varBlock[v].push_back(x.var());
    }

    listPU.clear();    
    s.decideAndComputeUnit(~l, listPU);

    for(auto &x : listPU)
    {
      if(m_markedVar[x.var()]) continue;
      
      m_markedVar[x.var()] = true;
      varBlock[v].push_back(x.var());
    }

    for(auto &x : varBlock[v]) m_markedVar[x] = false;
  }
} // extractVarBlock


/**
   Sort the indice list of a list of list in descending order regarding the size
   of the list.

   @param[in] varBlock, list of list.
   @param[out] indexsorted, the sorted indices.
 */
void AtMost1Extractor::computeSortedIndice(
    std::vector< std::vector<Var> > &varBlock,
    std::vector<unsigned> &indexSorted)
{
  indexSorted.clear();
  for(unsigned i = 0 ; i <= m_nbVar ; i++) indexSorted.push_back(i);

  struct MapVarBlock
  {
    std::vector< std::vector<Var> > &m_varBlock;
    
    MapVarBlock(std::vector< std::vector<Var> > varBlock) :
        m_varBlock(varBlock) {}

    bool operator() (int i, int j)
    {
      return m_varBlock[i].size() > m_varBlock[j].size();
    }
  };

  std::sort(indexSorted.begin(), indexSorted.end(), MapVarBlock(varBlock));
} // computeSortedIndice


/**
   Research equivalences in the set of variable v.

   @param[in] om, the spec manager.
   @param[in] v, the set of variables we search in.
   @param[out] equivVar, le resulting equivalences.
 */
void AtMost1Extractor::searchAtMost1(
    WrapperSolver &s,
    std::vector<Var> &vars,
    std::vector<AtMost1> &atMostList)
{ 
  // compute the list of binary block.
  std::vector< std::vector<Var> > varBlock;
  extractVarBlock(s, vars, varBlock);

  // sort the varblock regarding their size.
  std::vector<unsigned> indexSorted;
  computeSortedIndice(varBlock, indexSorted);

  
  for(auto &idx : indexSorted)
  {
    if(varBlock[idx].size() < 2) continue;
    
    for(auto &v : varBlock[idx]) std::cout << v << " ";
    std::cout << "\n";
  }
} // searchAtMost1

} // d4
