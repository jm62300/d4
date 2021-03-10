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

    int precision = vm["float-precision"].as<int>();
    std::ofstream ofs;
    ofs.setstate(std::ios_base::badbit);
    m_out << "c [CONSTRUCTOR] Create an external counter: " << "counting" << "\n";
    Counter<mpz::mpz_int> *counter = Counter<mpz::mpz_int>::makeCounter<mpz::mpz_int>(
        vm, m_problem, "counting", isFloat, precision, ofs);
    m_out << counter->count(ofs) << "\n";
  } // constructor


  /**
     Destructor.
   */
  ~ProjMCMethod()
  {
    delete m_problem;
  } // destructor


 private:


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
