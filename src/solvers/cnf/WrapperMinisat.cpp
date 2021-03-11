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

#include <iostream>
#include <typeinfo>       

#include "WrapperMinisat.hpp"

#include <src/problem/cnf/ProblemManagerCnf.hpp>

#include "minisat/Solver.hpp"
#include "minisat/SolverTypes.hpp"
#include "minisat/mtl/Vec.hpp"

namespace d4
{
/**
   This function initializes the SAT solver with a given problem.  Warning: we
   suppose that p is a CNF, otherwise a bad_cast exception is threw.

   @param[in] p, the problem we want to link with the SAT solver.
 */
void WrapperMinisat::initSolver(ProblemManager &p)
{
  try
  {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(p);

    // say to the solver we have pcnf.getNbVar() variables.
    while((unsigned)s.nVars() <= pcnf.getNbVar()) s.newVar();

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
  
  m_activeModel = false;
  m_needModel = false;
  setNeedModel(m_needModel);
} // initSolver


/**
   Call the SAT solver and return its result.

   @param[in] setOfvar, the variables we focus the solving process.
   
   \return true if the problem is SAT, false otherwise.
 */
bool WrapperMinisat::solve(std::vector<Var> &setOfVar)
{
  if(m_activeModel && m_needModel) return true;
  
  m_setOfVar_m.setSize(0);
  for(auto &v : setOfVar) m_setOfVar_m.push(v);
  s.rebuildWithConnectedComponent(m_setOfVar_m);  
  
  m_activeModel = s.solveWithAssumptions();  
  return m_activeModel;
} // solve


/**
   Call the SAT solver and return its result.

   \return true if the problem is SAT, false otherwise.
 */
bool WrapperMinisat::solve()
{
  s.rebuildWithAllVar();
  return s.solveWithAssumptions();
} // solve


/**
   Get the problem from the solver and store the result in pout.

   @param[out] pout, save the formulat in pout
 */
void WrapperMinisat::getSimplifiedFormula(ProblemManager &pout)
{
  try
  {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(pout);
    std::vector<std::vector<Lit> > &ret = pcnf.getClauses();
    ret.clear();

    for(int i = 0 ; i<s.trail.size() ; i++)
    {
      std::vector<Lit> cl;
      cl.push_back(Lit::makeLit(minisat::var(s.trail[i]), minisat::sign(s.trail[i])));
      ret.push_back(cl);
    }
    
    for(int i = 0 ; i<s.clauses.size() ; i++)
    {
      minisat::Clause &c = s.ca[s.clauses[i]];

      bool isSAT = false;
      std::vector<Lit> cl;
      for(int j = 0 ; j<c.size() && !isSAT ; j++)
      {
        if(s.isUndef(c[j]))
          cl.push_back(Lit::makeLit(minisat::var(c[j]), minisat::sign(c[j])));
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


/**
   An accessor on the activity of a variable.

   @param[in] v, the variable we want the activity.

   \return the activity of v.
 */
double WrapperMinisat::getActivity(Var v)
{
  return s.activity[v];
} // getActivity


/**
   Return the number of times the variable v occurs in a conflict.
   
   @param[in] v, the variable we want the activity.
   
   \return the number of times v occurs in a conflict.
 */
double WrapperMinisat::getCountConflict(Var v)
{
  return s.scoreActivity[v];
} // getCountConflict


/**
   Print out the trail on the standard output.
 */
void WrapperMinisat::showTrail()
{
  s.showTrail();
} // showTrail

/**
   An accessor on the polarity of a variable.

   @param[in] v, the variable we want the polarity.
 */
bool WrapperMinisat::getPolarity(Var v)
{
  return s.polarity[v];
} // getPolarity


/**
   Collect the unit literal from the affectation of the literal l to the
   formula.

   @param[in] l, the literal we want to branch on.
   @param[out] units, the unit literals

   \return true if assign l and propagate does not give a conflict, false otherwise.
 */
bool WrapperMinisat::decideAndComputeUnit(Lit l, std::vector<Lit> &units)
{
  minisat::Lit ml = minisat::mkLit(l.var(), l.sign());
  if(varIsAssigned(l.var()))
  {
    if(s.litAssigned(l.var()) != ml) return false;
    units.push_back(l);
    return true;
  }

  int posTrail = (s.trail).size();
  s.newDecisionLevel();
  s.uncheckedEnqueue(ml);
  minisat::CRef confl = s.propagate();

  if(confl != minisat::CRef_Undef) // unit literal
  {
    int bt;
    minisat::vec<minisat::Lit> learnt_clause;
    s.analyzeLastUIP(confl, learnt_clause, bt);
    s.cancelUntil(s.decisionLevel() - 1);
    assert(learnt_clause[0] == minisat::mkLit(l.var(), !l.sign()));
    s.insertClauseAndPropagate(learnt_clause);
    return false;
  }
      
  for(int j = posTrail ; j<s.trail.size() ; j++)
    units.push_back(Lit::makeLit(var(s.trail[j]), sign(s.trail[j])));
  s.cancelUntil(s.decisionLevel() - 1);
  return true;
} // decideAndComputeUnit


/**
   Fill the vector units with the literal l that are units such that l.var() is
   in component.

   @param[in] component, the set of variables we search for.
   @param[out] units, the place where we store the literals found.
 */
void WrapperMinisat::whichAreUnits(std::vector<Var> &component,
                                   std::vector<Lit> &units)
{
  for(auto &v : component)
  {
    if(!s.isAssigned(v)) continue;
    minisat::Lit l = s.litAssigned(v);
    units.push_back(Lit::makeLit(var(l), sign(l)));
  }
} // whichAreUnits


/**
   Check out if the given variable is assigned or not by the solver.

   @param[in] v, the variable we search for.

   \return true if the variable is assigned, false otherwise.
 */
bool WrapperMinisat::varIsAssigned(Var v)
{
  return s.isAssigned(v);
} // varIsAssigned

/**
   Restart the solver.
 */
void WrapperMinisat::restart()
{
  s.cancelUntil(0);
} // restart


/**
   Transfer to the solver the fact we have a set of assumption variables we want
   to consider.

   @param[in] assums, the set of assumptions
 */
void WrapperMinisat::setAssumption(std::vector<Lit> &assums)
{
  minisat::vec<minisat::Lit> &assumptions = s.assumptions;
  assumptions.clear();
  for(auto &l : assums) pushAssumption(l);
} // setAssumption


/**
   Print out the assumption.

   @param[in] out, the stream where is print the assumption.
 */
void WrapperMinisat::displayAssumption(std::ostream &out)
{
  minisat::vec<minisat::Lit> &assumptions = s.assumptions;
  for(int i = 0 ; i<assumptions.size() ; i++)
  {
    minisat::Lit l = assumptions[i];
    std::cout << (minisat::sign(l) ? "-" : "") << minisat::var(l) << " ";
  }
  std::cout << "\n";
}// displayAssumption


/**
   Ask for the model.

   @param[in] b, a boolean value to ask the solver to get the model.
 */
void WrapperMinisat::setNeedModel(bool b)
{
  m_needModel = b;
  s.setNeedModel(b);
} // setNeedModel


/**
   Push a new assumption.

   @param[in] l, the literal we want to push.
 */
void WrapperMinisat::pushAssumption(Lit l)
{
  minisat::Lit ml = minisat::mkLit(l.var(), l.sign());
  m_activeModel = m_activeModel && !s.isAssigned(var(ml));

  (s.assumptions).push(ml);

  if(m_activeModel && m_needModel)
  {
    m_activeModel = s.litTrueInLastModel(ml);
    if(m_activeModel)
    {
      assert(s.decisionLevel() == s.assumptions.size() - 1);
      s.newDecisionLevel();
      assert(!s.isAssigned(var(ml)));
      s.uncheckedEnqueue(ml);
      [[maybe_unused]] minisat::CRef cref = s.propagate();
      assert(cref == minisat::CRef_Undef);
    }
  }
} // pushAssumption


/**
   Remove the last assumption and cancelUntil.
 */
void WrapperMinisat::popAssumption()
{
  (s.assumptions).pop();
  (s.cancelUntil)((s.assumptions).size());  
} // popAssumption


} // d4
