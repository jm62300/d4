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

#ifndef d4_src_solvers_cnf_minisat_WrapperSolver_hpp
#define d4_src_solvers_cnf_minisat_WrapperSolver_hpp

#include <boost/program_options.hpp>

#include <src/problem/ProblemTypes.hpp>
#include <src/problem/ProblemManager.hpp>

#include "ActivityManager.hpp"
#include "PolarityManager.hpp"

namespace d4
{
namespace po = boost::program_options;
class WrapperSolver : public ActivityManager, public PolarityManager
{
  private:

 public:
  static WrapperSolver *makeWrapperSolver(po::variables_map &vm);
  static WrapperSolver *makeWrapperSolverPreproc(po::variables_map &vm);

  virtual ~WrapperSolver(){}
  virtual void initSolver(ProblemManager &p) = 0;
  virtual bool solve() = 0;
  virtual void getSimplifiedFormula(ProblemManager &p) = 0;
  virtual void restart() = 0;
  virtual void setAssumption(std::vector<Lit> &assums) = 0;
  virtual void pushAssumption(Lit l) = 0;
  virtual void popAssumption() = 0;
  virtual bool varIsAssigned(Var v) = 0;
  virtual void setNeedModel(bool b) = 0;
  virtual void showTrail() = 0;

  // this function returns false if the propagation gives a conflict.
  virtual bool decideAndComputeUnit(Lit l, std::vector<Lit> &units) = 0;

  // we only consider setOfvar as input variable.
  virtual void inputVar(std::vector<Var> &setOfVar) = 0;
  
  virtual void whichAreUnits(std::vector<Var> &component,
                             std::vector<Lit> &units) = 0;
  
};
} // d4

#endif
