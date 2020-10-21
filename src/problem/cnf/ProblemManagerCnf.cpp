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

#include "ProblemManagerCnf.hpp"
#include "ParserDimacs.hpp"

namespace d4
{
/**
   Constructor.

   @param[in] nameFile, parse the instance from a file
 */
ProblemManagerCnf::ProblemManagerCnf(std::string &nameFile)
{
  ParserDimacs parser;
  nbVars = parser.parse_DIMACS(nameFile, clauses);
} // constructor


/**
   Constructor.
   Construct an empty formula.
 */
ProblemManagerCnf::ProblemManagerCnf()
{
  nbVars = 0;
} // constructor


/**
   Destructor.
 */
ProblemManagerCnf::~ProblemManagerCnf()
{
  clauses.clear();
  nbVars = 0;
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
