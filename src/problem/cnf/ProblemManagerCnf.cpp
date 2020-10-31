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
  m_nbVar = parser.parse_DIMACS(nameFile, m_clauses);
} // constructor


/**
   Constructor.
   Construct an empty formula.
 */
ProblemManagerCnf::ProblemManagerCnf()
{
  m_nbVar = 0;
} // constructor


/**
   Constructor.
   Construct an empty formula.
 */
ProblemManagerCnf::ProblemManagerCnf(int nbVar) 
{
  m_nbVar = nbVar;
} // constructor


/**
   Destructor.
 */
ProblemManagerCnf::~ProblemManagerCnf()
{
  m_clauses.clear();
  m_nbVar = 0;
} // destructor


/**
   Display the problem.

   @param[out] out, the stream where the messages are redirected.
 */
void ProblemManagerCnf::display(std::ostream &out)
{
  out << "p cnf " << m_nbVar << " " << m_clauses.size() << "\n";
  for(auto cl : m_clauses)
  {
    for(auto &l : cl) out << l << " ";
    out << "0\n";
  }
} // diplay

}
