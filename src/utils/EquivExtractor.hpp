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

#ifndef d4_src_utils_EquivExtractor_hpp
#define d4_src_utils_EquivExtractor_hpp

#include <vector>

#include <src/solvers/WrapperSolver.hpp>
#include <src/problem/ProblemTypes.hpp>

namespace d4
{
class EquivExtractor
{
 private:
  std::vector<bool> markedVar;
  std::vector<bool> markedVarInter;
  
 public:
  EquivExtractor(){;} // empty constructor 
  EquivExtractor(int nbVar);  
  void initEquivExtractor(int nbVar);  
  bool interCollectUnit(WrapperSolver &s, Var v, std::vector<Var> &listVarPU);
  void searchEquiv(WrapperSolver &s,
                   std::vector<Var> &v,
                   std::vector< std::vector<Var> > &equivVar);
};
} // d4

#endif
