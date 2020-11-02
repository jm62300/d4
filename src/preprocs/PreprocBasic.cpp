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

#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "PreprocBasic.hpp"

namespace d4
{

/**
   The constructor. 

   @param[in] vm, the options used (solver).
 */
PreprocBasic::PreprocBasic(po::variables_map &vm)
{
  ws = WrapperSolver::makeWrapperSolverPreproc(vm);
} // constructor


/**
   Destructor.
 */
PreprocBasic::~PreprocBasic()
{
  delete ws;
} // destructor


/**
   The preprocessing itself.

   @param[out] p, the problem we want to preprocess.
 */
ProblemManager *PreprocBasic::run(ProblemManager &pin)
{
  ws->initSolver(pin);
  ProblemManagerCnf *pout = new ProblemManagerCnf(pin.getNbVar());
  
  try
  {    
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(*pout); 
    
    if(!ws->solve()) // p is UNSAT 
    {
      std::vector<std::vector<Lit> > &ret = pcnf.getClauses();
      ret.clear();
    
      std::vector<Lit> cl;
      Lit l = Lit(1, false);
    
      cl.push_back(l);
      ret.push_back(cl);
    
      cl[0] = l.neg();
      ret.push_back(cl);
    }
    else // SAT: extract the clauses from the solver.
    {
      ws->getSimplifiedFormula(*pout);
    }
  }
  catch (std::bad_cast& bc)
  {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }

  return pout;
} // run

}
