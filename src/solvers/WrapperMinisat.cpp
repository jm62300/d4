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

#include "WrapperMinisat.hpp"

#include <iostream>
#include <typeinfo>       

#include "../problem/cnf/ProblemManagerCnf.hpp"
#include "minisat/Solver.hpp"
#include "minisat/SolverTypes.hpp"
#include "minisat/mtl/Vec.hpp"

namespace d4
{
/**
   This function initializes the SAT solver with a given problem.
   Warning: we suppose that p is a CNF, otherwise a bad_cast exception is threw.

   @param[in] p, the problem we want to link with the SAT solver.
 */
void WrapperMinisat::initSolver(ProblemManager &p)
{
  try
  {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(p);

    // say to the solver we have pcnf.getNbVar() variables.
    while(s.nVars() <= pcnf.getNbVar()) s.newVar();

    // load the clauses
    std::vector<std::vector<Lit>> &clauses = pcnf.getClauses();
    for(auto &cl : clauses)
    {
      minisat::vec<minisat::Lit> lits;
      for(auto &l : cl) lits.push(minisat::mkLit(l.var(), l.sign()));
      s.addClause(lits);
    }
  }
  catch (std::bad_cast& bc)
  {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }
} // initSolver


/**
   Call the SAT solver and return its result.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperMinisat::solve()
{
  return s.solveWithAssumptions();
} // solve


/**
   Get the problem from the solver and store the result in p.
 */
void WrapperMinisat::getSimplifiedFormula(ProblemManager &p)
{
  try
  {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(p);      
    std::vector<std::vector<Lit> > &ret = pcnf.getClauses();
    ret.clear();

    for(int i = 0 ; i<s.trail.size() ; i++)
    {
      std::vector<Lit> cl;
      cl.push_back(Lit(minisat::var(s.trail[i]), minisat::sign(s.trail[i])));
      ret.push_back(cl);
    }

    for(int i = 0 ; s.clauses.size() ; i++)
    {
      minisat::Clause &c = s.ca[s.clauses[i]];

      bool isSAT = false;
      std::vector<Lit> cl;
      for(int j = 0 ; j<c.size() && !isSAT ; i++)
      {
        if(s.isUndef(c[j]))
          cl.push_back(Lit(minisat::var(c[j]), minisat::sign(c[j])));
        isSAT = s.isSAT(c[j]);
      }

      if(!isSAT) ret.push_back(cl);
    }
  }
  catch (std::bad_cast& bc)
  {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }
} // getSimplifiedFormula


} // d4
