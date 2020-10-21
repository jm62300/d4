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

#ifndef d4_src_heuristics_ScoringMethod_hpp
#define d4_src_heuristics_ScoringMethod_hpp

#include <vector>
#include <boost/program_options.hpp>

#include <solvers/WrapperSolver.hpp>
#include <problem/ProblemTypes.hpp>
#include <specs/SpecManager.hpp>

namespace d4
{
namespace po = boost::program_options;
class ScoringMethod
{    
 public: 
  static ScoringMethod *makeScoringMethod(po::variables_map &vm, SpecManager &p);
  virtual ~ScoringMethod(){;}
  virtual double computeScore(Var v) = 0;
};
}

#endif
