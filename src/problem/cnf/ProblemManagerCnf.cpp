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

#include "../../preprocs/PreprocManager.hpp"
#include "CnfOccurrenceManager.hpp"
#include "ProblemManagerCnf.hpp"
#include "ParserDimacs.hpp"

namespace d4
{
/**
   Constructor. Take as arguments on the options.

   @param[in] vm, the arguments on the command line.
 */
ProblemManagerCnf::ProblemManagerCnf(po::variables_map &vm)
{
  ParserDimacs parser;
  nbVars = parser.parse_DIMACS(vm["input"].as<std::string>(), clauses);

  // call the preproc and collect clauses and unit literals.
  PreprocManager *preproc = PreprocManager::makePreprocManager(vm);
  assert(preproc);
  preproc->run(*this);
  delete preproc;
  
  // add the clauses+units to the solver.
  ws = WrapperSolver::makeWrapperSolver(vm);
  assert(ws);
  ws->initSolver(*this);
        
  // initialize the occurence manager: clauses + units
  om = CnfOccurrenceManager::makeCnfOccurrenceManager(vm, *this);
  assert(om);
  
} // constructor


/**
   Destructor.
 */
ProblemManagerCnf::~ProblemManagerCnf()
{
  clauses.clear();
  nbVars = 0;

  delete ws;
} // destructor


/**
   Display the problem.

   @param[out] out, the stream where the messages are redirected.
 */
void ProblemManagerCnf::display(std::ostream &out)
{
  out << "p cnf " << nbVars << " " << clauses.size() << "\n";
  for(auto cl : clauses)
  {
    for(auto &l : cl) out << l << " ";
    out << "0\n";
  }
} // diplay

}
