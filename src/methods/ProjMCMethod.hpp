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
  std::ostream m_out;

  ProblemManager *m_problem;

  std::vector<bool> m_isProjecectVar;
 public:

  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  ProjMCMethod(
      po::variables_map &vm,
      bool isFloat, 
      ProblemManager *initProblem) : m_out(nullptr)
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

    // mark the projected variables.
    m_isProjecectVar.resize(m_problem->getNbVar() + 1, false);
    m_out << "c\n" << "c [PROJECTED VARIABLES] list: ";
    std::vector<Var> &selected = m_problem->getSelectedVar();
    std::sort(selected.begin(), selected.end());
    for(auto v : selected)
    {
      m_out << v << " ";
      m_isProjecectVar[v] = true;
    }
    m_out << "\n" << "c\n";


    std::vector<std::vector<Lit>> projClause, nprojClause, mix;
    partitionFormula(m_problem, m_isProjecectVar, projClause, nprojClause, mix);
    
    int precision = vm["float-precision"].as<int>();
    std::ofstream ofs;
    ofs.setstate(std::ios_base::badbit);
    m_out << "c [CONSTRUCTOR] Create an external counter: " << "counting" << "\n";
    Counter<mpz::mpz_int> *counter = Counter<mpz::mpz_int>::makeCounter<mpz::mpz_int>(
        vm, m_problem, "counting", isFloat, precision, ofs);

    m_out << counter->count(ofs) << "\n";
    
    std::vector<Lit> assums;
    assums.push_back(Lit::makeLit(1, false));
    m_out << counter->count(assums, ofs) << "\n";

    assums[0] = ~assums[0];
    m_out << counter->count(assums, ofs) << "\n";
  } // constructor


  /**
     Destructor.
   */
  ~ProjMCMethod()
  {
    delete m_problem;
  } // destructor


 private:

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
  void partitionFormula(ProblemManager *problem,
                        std::vector<bool> &isProjectVar,
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
        if(m_isProjecectVar[l.var()]) nbp++; else nbn++;
        if(nbp && nbn) break;
      }

      if(nbp && !nbn) projClause.push_back(cl);
      else if(!nbp && nbn) nprojClause.push_back(cl);
      else mix.push_back(cl);
    }
  } // partitionFormula
  

 public:
  
  /**
     Run the DPLL style algorithm with the operation manager.

     @param[in] vm, the set of options.
   */
  void run(po::variables_map &vm)
  {
    std::cout << "in construction\n";
  } // run
};
} // d4
