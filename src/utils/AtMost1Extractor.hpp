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
#pragma once

#include <vector>

#include "src/solvers/WrapperSolver.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4
{
struct AtMost1 
{  
  std::vector<Var> list;

  void display()
  {
    for(auto &l : list) std::cout << l << " ";
    std::cout << "\n";
  }
};

class AtMost1Extractor
{
 private:
  std::vector<bool> m_markedVar;
  std::vector<unsigned> m_stamp;

  unsigned m_nbVar;

  void extractVarBlock(WrapperSolver &s,
                       std::vector<Var> &vars,
                       std::vector< std::vector<Var> > &varBlock);
  
  void computeSortedIndice(std::vector< std::vector<Var> > &varBlock,
                           std::vector<unsigned> &indexSorted);
  
 public:
  AtMost1Extractor(){;} // empty constructor 
  AtMost1Extractor(int nbVar);  
  void init(int nbVar);
  
  void searchAtMost1(WrapperSolver &s,
                     std::vector<Var> &v,
                     std::vector<AtMost1> &atMostList);
};
} // d4
