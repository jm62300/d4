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

#include "WrapperSolver.hpp"
#include "cnf/WrapperMinisat.hpp"

namespace d4
{
/**
   Wrapper to get a solver able to solve the input problem for the
   compilation/counting problems.

   @param[in] vm, the options.
 */
WrapperSolver *WrapperSolver::makeWrapperSolver(po::variables_map &vm)
{
  std::string in = vm["input"].as<std::string>();
  std::string extension = in.substr(in.find_last_of(".") + 1);
  std::string s = vm["solver"].as<std::string>();

  if(extension == "cnf" || extension == "dimacs")
  {    
    if(s == "minisat") return new WrapperMinisat();
    return NULL;
  }
  return NULL;
} // makeWrapperSolver


/**
   Wrapper to get a solver able to solve the input problem for the preprocessing
   step.

   @param[in] vm, the options.
 */
WrapperSolver *WrapperSolver::makeWrapperSolverPreproc(po::variables_map &vm)
{
  std::string s = vm["preproc-solver"].as<std::string>();

  if(s == "minisat") return new WrapperMinisat();
  
  return NULL;
} // makeWrapperSolverPreproc


}
