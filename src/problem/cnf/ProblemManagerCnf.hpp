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

#ifndef d4_problem_ProblemManagerCnf_hpp
#define d4_problem_ProblemManagerCnf_hpp

#include <boost/program_options.hpp>

#include "../ProblemTypes.hpp"
#include "../ProblemManager.hpp"
#include "../../solvers/WrapperSolver.hpp"

namespace d4
{
namespace po = boost::program_options;

class ProblemManagerCnf : public ProblemManager
{
 private:
  int nbVars;
  std::vector< std::vector<Lit> > clauses;
  WrapperSolver *ws;
  
 public:
  ProblemManagerCnf(po::variables_map &vm);
  ~ProblemManagerCnf();

  int getNbVar(){return nbVars;}
  void display(std::ostream &out);

  std::vector< std::vector<Lit> > &getClauses(){return clauses;}
};
  
} // d4

#endif
