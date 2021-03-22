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

#include <iostream>
#include <iomanip>
#include <ctime>
#include <unordered_map>
#include <boost/program_options.hpp>

#include "src/preprocs/PreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "MethodManager.hpp"

namespace d4
{
namespace po = boost::program_options;
namespace mpz = boost::multiprecision;

template<class T>
class ProjMCMethod : public MethodManager
{
 private:
  struct CoupleNotProjClauseSelector
  {
    std::vector<Lit> clause;
    Lit selector;

    void display()
    {
      std::cout << selector << " : ";
      for(const auto &l : clause) std::cout << l << " ";
      std::cout << "\n";
    }
  };
  
  std::ostream m_out;
  std::ostream m_outCounter;
  
  ProblemManager *m_problem;
  std::vector<Lit> m_selectors;
  std::vector<std::vector<Lit>> selectorToProjClause;
  std::vector<bool> m_isProjectedVar;
  std::vector<int> m_marked;
  std::unordered_map<std::string, Lit> mapProjClSelector;
  std::vector<CoupleNotProjClauseSelector> notProjClauses;

  SpecManager *m_specs;
  WrapperSolver *m_solver;
  Counter<T> *m_counter;
 public:

  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  ProjMCMethod(
      po::variables_map &vm,
      bool isFloat, 
      ProblemManager *initProblem) : m_out(nullptr), m_outCounter(nullptr)
  {
    // init the output stream
    m_out.copyfmt(std::cout);                          
    m_out.clear(std::cout.rdstate());           
    m_out.basic_ios<char>::rdbuf(std::cout.rdbuf());

    // we call the preproc and we generate the problem used after.
    PreprocManager *preproc = PreprocManager::makePreprocManager(vm, m_out);
    assert(preproc);
    m_problem = preproc->run(*initProblem);
    m_out << "c [PREPROCESSED INPUT] \033[4m\033[32mStatistics about the preprocessed formula\033[0m\n";
    m_problem->displayStat(m_out, "c [PREPROCESSED INPUT] ");
    m_out << "c\n";
    assert(m_problem);
    delete preproc;

    // mark the projected variables.
    m_isProjectedVar.resize(m_problem->getNbVar() + 1, false);
    m_out << "c\n" << "c [PROJECTED VARIABLES] list: ";
    std::vector<Var> &selected = m_problem->getSelectedVar();
    std::sort(selected.begin(), selected.end());
    for(auto v : selected)
    {
      m_out << v << " ";
      m_isProjectedVar[v] = true;
    }
    m_out << "\n" << "c\n";

    std::vector<std::vector<Lit>> projClause, nprojClause, mix;
    partitionFormula(m_problem, m_isProjectedVar, projClause, nprojClause, mix);

    // split the mixed clauses.
    unsigned idxVar = m_problem->getNbVar() + 1;
    manageMixedClauses(mix, m_isProjectedVar, projClause, nprojClause,
                       m_selectors, idxVar);

    // prepare the SAT solver.
    std::vector<std::vector<Lit>> satSolverClauses = projClause;
    for(auto &cl : nprojClause) satSolverClauses.push_back(cl);
    initSatSolver(vm, m_problem, satSolverClauses, idxVar - 1);

    // prepare the counter.
    initCounter(vm, m_problem, isFloat, projClause, idxVar - 1);
    m_marked.resize(idxVar + 1, -1);
  } // constructor


  /**
     Destructor.
   */
  ~ProjMCMethod()
  {
    if(m_counter) delete m_counter;
    if(m_solver) delete m_solver;
    if(m_specs) delete m_specs;
    delete m_problem;
  } // destructor


 private:

  /**
     Init the counter with the projected clauses.

     @param[in] vm, the option to create the SAT solver.     
     @param[in] problem, the input problem (only used to get information about
     weight).
     @param[in] isFloat, specify if the weight are float or int.
     @param[in] clauses, the set of clauses.
     @param[in] nbVar, the number of variables of the formula.
   */
  void initCounter(
      po::variables_map &vm,
      ProblemManager *problem,
      bool isFloat, 
      std::vector<std::vector<Lit>> &clauses,
      unsigned nbVar)
  {
    int precision = vm["float-precision"].as<int>();
#if DEBUG
    m_outCounter.copyfmt(m_out);
    m_outCounter.clear(m_out.rdstate());
    m_outCounter.basic_ios<char>::rdbuf(m_out.rdbuf());
#else    
    m_outCounter.setstate(std::ios_base::badbit);
#endif
    // init the problem we will pass to the counter.
    std::vector<double> weightLit((nbVar + 1) << 1, 1);
    std::vector<double> weightVar(nbVar + 1, 2);
    
    std::vector<double> &problemWeightLit = problem->getWeightLit();
    for(unsigned i = 0 ; i<problemWeightLit.size() ; i++)
      weightLit[i] = problemWeightLit[i];

    std::vector<double> &problemWeightVar = problem->getWeightVar();
    for(unsigned i = 0 ; i<problemWeightVar.size() ; i++)
    {
      if(m_isProjectedVar[i]) weightVar[i] = problemWeightVar[i];
      else weightVar[i] = 1; 
    }

    std::vector<Var> emptySelectedVar;    
    ProblemManagerCnf p(nbVar, weightLit, weightVar, emptySelectedVar);
    p.setClauses(clauses);
    
    // create the counter.
    m_out << "c [CONSTRUCTOR] Create an external counter: " << "counting" << "\n";
    m_counter = Counter<T>::makeCounter(
        vm, &p, "counting", isFloat, precision, m_outCounter);
  } // initCounter
  

  /**
     Init the SAT solver with a set of clauses (actually two sets).  Only deals
     with CNF formula.

     @param[in] vm, the option to create the SAT solver.     
     @param[in] problem, the input problem (only used to get information about
     weight).
     @param[in] clauses, the set of clauses.
     @param[in] nbVar, the number of variables of the formula.
   */
  void initSatSolver(
      po::variables_map &vm,
      ProblemManager *problem,
      std::vector<std::vector<Lit>> &clauses,
      unsigned nbVar)
  {
    m_solver = WrapperSolver::makeWrapperSolver(vm, m_out);
    assert(m_solver);
    
    // prepare the weight vectors and init the problem.
    std::vector<double> weightLit((nbVar + 1) << 1, 1);
    std::vector<double> weightVar(nbVar + 1, 2);
    
    std::vector<double> &problemWeightLit = problem->getWeightLit();
    for(unsigned i = 0 ; i<problemWeightLit.size() ; i++)
      weightLit[i] = problemWeightLit[i];

    std::vector<double> &problemWeightVar = problem->getWeightVar();
    for(unsigned i = 0 ; i<problemWeightVar.size() ; i++)
      weightVar[i] = problemWeightVar[i];
    
    ProblemManagerCnf p(nbVar, weightLit, weightVar, problem->getSelectedVar());
    p.setClauses(clauses);
    m_solver->initSolver(p);

    // ask for the witness.
    m_solver->setNeedModel(true);

    // prepare the spec manager.
    m_specs = SpecManager::makeSpecManager(vm, p, m_out);
  } // initSatSolver

  
  /**
     Partition the formula in tree sets regarding the the clauses contain or not
     projected variable.

     @param[in] prolem, the problem we want to partition.
     @param[in] isProjectvar, boolean vector that specifies the projected
     variables.     
     @param[out] projClause, the clauses that only contain projected variable.     
     @param[out] nprojClause, the clauses that do not contain projected
     variable.     
     @param[out] mix, the clauses that contain both projected variable and not.
     projected variables.
   */
  void partitionFormula(
      ProblemManager *problem,
      std::vector<bool> &isProjectedVar,
      std::vector<std::vector<Lit>> &projClause,
      std::vector<std::vector<Lit>> &nprojClause,
      std::vector<std::vector<Lit>> &mix)
  {
    ProblemManagerCnf *cnf = static_cast<ProblemManagerCnf *>(problem);
    std::vector<std::vector<Lit>> &clauses = cnf->getClauses();

    for(auto &cl : clauses)
    {
      unsigned nbp = 0, nbn = 0;
      for(auto l : cl)
      {
        if(isProjectedVar[l.var()]) nbp++; else nbn++;
        if(nbp && nbn) break;
      }

      if(nbp && !nbn) projClause.push_back(cl);
      else if(!nbp && nbn) nprojClause.push_back(cl);
      else mix.push_back(cl);
    }
  } // partitionFormula


  /**
     Manage the mixed clauses by adding a selector in order to seperate each
     clause between projected and not projected variables.

     @param[int] mix, the clauses that contain both projected variable and not.
     projected variables.     
     @param[in] isProjectvar, boolean vector that specifies the projected
     variables.     
     @param[out] projClause, the clauses that only contain projected variable.     
     @param[out] nprojClause, the clauses that do not contain projected
     variable.
     @param[out] nextVar, the index of the next available variable.
     
     @param[out] selectors, the added selectors literals (that can be used to
     activate the projected part - ¬s v C).
   */
  void manageMixedClauses(
      std::vector<std::vector<Lit>> &mix,
      std::vector<bool> &isProjectedVar,
      std::vector<std::vector<Lit>> &projClause,
      std::vector<std::vector<Lit>> &nprojClause,
      std::vector<Lit> &selectors,
      unsigned &nextVar)
  {
    for(auto &cl : mix)
    {
      std::vector<Lit> clp, clnp;
      std::string key = "";
      for(auto &l : cl)
        if(!isProjectedVar[l.var()]) clnp.push_back(l);
        else
        {
          key += "." + std::to_string(l.intern());
          clp.push_back(l);
        }

      Lit s = lit_Undef;
      if(mapProjClSelector.count(key) > 0) s = mapProjClSelector[key];
      else
      {        
        s = Lit::makeLitTrue(nextVar++);
        mapProjClSelector[key] = s;
        
        if((int) selectorToProjClause.size() <= s.var())
          selectorToProjClause.resize(s.var() + 1, std::vector<Lit>());
        selectorToProjClause[s.var()] = clp;

        clp.push_back(s);
        projClause.push_back(clp);

        m_isProjectedVar.resize(nextVar);
        m_isProjectedVar[s.var()] = false;
      }
      
      // link the selector to the not projected part of the considered clause.
      notProjClauses.push_back({clnp, s});
      clnp.push_back(~s);
      nprojClause.push_back(clnp);      
    }
  } // manageMixedClauses


#define TEST 0
  
  /**
     Extract the selector that correspond to the non projected clauses that are
     falsified by a given interpretation.

     @param[in] model, the interpretation we check.
     @param[out] selector, the returned selector.
   */
  void extractSelectorFalsifiedNProj(
      std::vector<lbool> &model,
      std::vector<Lit> &selector)
  {  
#if TEST
    for(unsigned i = 0 ; i<model.size() ; i++)
    {
      if(model[i] == l_Undef) continue;
      std::cout << i << "(" << (model[i] == l_True) << ")" << " ";
    }
    
    std::cout << "\n";
#endif
    
    for(auto &value : notProjClauses)
    {
      if(m_solver->isInAssumption(value.selector.var())) continue;
      
      const std::vector<Lit> &cl = value.clause;
      bool isSAT = false;
      for(auto &l : cl)
      {
        if(model[l.var()] == l_Undef) continue;
        else if(l.sign()) isSAT = model[l.var()] == l_False;
        else isSAT = model[l.var()] == l_True;

        if(isSAT) break;
      }

      Lit s = isSAT ? value.selector : ~value.selector;
      if(s.sign() && m_marked[s.var()] != -1) selector[m_marked[s.var()]] = s;
      else if(m_marked[s.var()] == -1)
      {
        m_marked[s.var()] = selector.size();
        selector.push_back(s);
      }      
    }

    // reinit m_marked.
    for(auto &l : selector) m_marked[l.var()] = -1;
  } // extractSelectorFalsifiedNProj


  /**
     Expel from a list of variables, and a list of literals, the elements that
     are not belonging to the projected set of variables.

     @param[out] litList, the list of literals we want to refine.
     @param[out] varList, the list of variables want to refine.
   */
  void expelNoProjectedElement(
      std::vector<Lit> &litList,
      std::vector<Var> &varList)
  {
    // only keep the projected.
    unsigned j = 0;
    for(unsigned i = 0 ; i<litList.size() ; i++)
      if(m_isProjectedVar[litList[i].var()]) litList[j++] = litList[i];
    litList.resize(j);

    j = 0;    
    for(unsigned i = 0 ; i<varList.size() ; i++)
      if(m_isProjectedVar[varList[i]]) varList[j++] = varList[i];
    varList.resize(j);
  } // expelNoProjectedElement

  
  /**
     Compute the number of model on the projected variables.

     @param[in] setOfVar, the set of variableswe consider.
     @param[in] out, the stream where are printed the logs.

     \return the number of models.
   */
  T compute_(
      std::vector<Var> &setOfVar,
      std::ostream &out)
  {
    unsigned initSizeAssumption = m_solver->sizeAssumption();
    
#if TEST
    std::cout << "solver assumption\n";
    m_solver->displayAssumption(m_out);
#endif
    if(!m_solver->solve(setOfVar))
    {
#if TEST
      std::cout << "UNSAT\n";
#endif
      return T(0);
    }

    // collect the selectors of the unsatisfied non projected clauses.
    std::vector<Lit> selector;
    extractSelectorFalsifiedNProj(m_solver->getModel(), selector);

    // collect unit literals
    std::vector<Lit> unitLits;
    m_solver->whichAreUnits(setOfVar, unitLits);
    m_specs->preUpdate(unitLits);

    // add the unit in the assumption.
    for(auto &l : unitLits)
    {
      assert(!m_solver->isInAssumption(~l));
      if(!m_solver->isInAssumption(l)) m_solver->pushAssumption(l);
    }

    std::vector<Var> reallyPresent, freeVariable;
    std::vector< std::vector<Var> > varConnected;
    int nbComponent = m_specs->computeConnectedComponent(
        varConnected, setOfVar, freeVariable, reallyPresent);
#if TEST
    std::cout << "Really present: ";
    for(auto &v : reallyPresent) std::cout << v << " ";
    std::cout << "\n";

    std::cout << "Unit: ";
    for(auto &l : unitLits) std::cout << l << " ";
    std::cout << "\n";

    std::cout << "free: ";
    for(auto &v : freeVariable) std::cout << v << " ";
    std::cout << "\n";
#endif
    // extract the projected variables.
    std::vector<Var> projectSetOfVar;
    for(auto &v : reallyPresent)
      if(m_isProjectedVar[v]) projectSetOfVar.push_back(v);    
    
    T ret = 1;
    if(nbComponent && projectSetOfVar.size())
    {
      // prepare the next assumption.
      std::vector<Lit> nextAssums(m_solver->getAssumption());
      for(auto &s : selector)
      {
        assert(!m_solver->isInAssumption(~s));
        if(!m_solver->isInAssumption(s)) nextAssums.push_back(s);
      }

#if TEST
      std::cout << "assums: ";
      for(auto &l : nextAssums) std::cout << l << " ";
      std::cout << "\n";
      
      std::cout << "selector: ";
      for(auto &l : selector) std::cout << l << " ";
      std::cout << "\n";
#endif
      ret = m_counter->count(reallyPresent, nextAssums, m_outCounter);
#if TEST
      std::cout << ret << " <<<\n";
#endif
      // reajust the selectors by only keeping the negative.
      unsigned j = 0;
      for(unsigned i = 0 ; i<selector.size() ; i++)
        if(selector[i].sign()) selector[j++] = selector[i];
      selector.resize(j);
      
      for(auto &s : selector)
      {
#if TEST
        std::cout << "considered selector " << s << "\n";
#endif
        unsigned countPushInAssumption = 0;
        auto &cl = selectorToProjClause[s.var()];
        bool isUnsat = false;
        for(auto &l : cl)
        {
          if(m_solver->isInAssumption(l))
          {
            isUnsat = true;
            break;
          }

          if(!m_solver->isInAssumption(~l))
          {
            m_solver->pushAssumption(~l);
            countPushInAssumption++;
          }
        }
        
        if(!isUnsat) ret += compute_(reallyPresent, out);
#if TEST
        std::cout << "Pop assumption : " << cl.size() << "\n";
        m_solver->displayAssumption(m_out);
#endif        
        m_solver->popAssumption(countPushInAssumption);
#if TEST
        std::cout << "Pop assumption\n";
        m_solver->displayAssumption(m_out);
#endif        
        if(m_solver->isInAssumption(~s)) break; // UNSAT.
        if(!m_solver->isInAssumption(s)) m_solver->pushAssumption(s);
      }
#if TEST
      std::cout << "Pop assumption selector: " << selector.size() << "\n";
      m_solver->displayAssumption(m_out);
#endif      
      m_solver->popAssumption(selector.size());
#if TEST
      std::cout << "Pop assumption selector\n";
      m_solver->displayAssumption(m_out);
#endif
    }
#if TEST
    else
    {
      std::cout << ret << " >>>\n";
    }
#endif
    
    m_specs->postUpdate(unitLits);
    expelNoProjectedElement(unitLits, freeVariable);

#if TEST
    std::cout << "count free and units: " 
              << m_problem->computeWeightUnitFree<T>(unitLits, freeVariable)
              << "\n";
    std::cout << "ret = " << ret << "\n";
#endif

    m_solver->popAssumption(m_solver->sizeAssumption() - initSizeAssumption);
    return ret * m_problem->computeWeightUnitFree<T>(unitLits, freeVariable);
  } // compute_

  
  /**
     Prepare the computed process.

     @param[in] out, the stream used to print the information.
   */
  T compute(std::ostream &out)
  {
    std::vector<Var> setOfVar ;
    for(int i = 1 ; i <= m_specs->getNbVariable() ; i++) setOfVar.push_back(i);
    return compute_(setOfVar, out);
  } // compute
  
  
 public:
  
  /**
     Run the DPLL style algorithm with the operation manager.

     @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm)
  {
    T res = compute(m_out);
    std::cout << "s " << res << "\n";
  } // run
};
} // d4
