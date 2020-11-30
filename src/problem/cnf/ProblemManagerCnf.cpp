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
  m_nbVar = parser.parse_DIMACS(nameFile, m_clauses, m_weightLit, m_selected);

  m_weightVar.resize(m_nbVar + 1, 0);
  for(unsigned i = 0 ; i <= m_nbVar ; i++)
    m_weightVar[i] = m_weightLit[i << 1] + m_weightLit[(i << 1) + 1];
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

   @param[in] nbVar, the number of variables.
   @param[in] weightLit, the weights associate with the literals.   
   @param[in] weightVar, the weights associate with the variables (sum of weight
   of the lit)
 */
ProblemManagerCnf::ProblemManagerCnf(int nbVar,
                                     std::vector<double> &weightLit,
                                     std::vector<double> &weightVar,
                                     std::vector<Var> &selected) 
{
  m_nbVar = nbVar;
  m_weightLit = weightLit;
  m_weightVar = weightVar;
  m_selected = selected;
  m_isUnsat = false;
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


/**
   Print out some statistic about the problem. Each line will start with the
   string startLine given in parameter.

   @param[in] out, the stream where the messages are redirected.
   @param[in] startLine, each line will start with this string.
 */
void ProblemManagerCnf::displayStat(std::ostream &out, std::string startLine)
{
  unsigned nbLits = 0;
  unsigned nbBin = 0;
  unsigned nbTer = 0;
  unsigned nbMoreThree = 0;

  for(auto &c :  m_clauses)
  {
    nbLits += c.size();
    if(c.size() == 2) nbBin++;
    if(c.size() == 3) nbTer++;
    if(c.size() > 3) nbMoreThree++;
  }
  
  out << startLine << "Number of variables: " << m_nbVar << "\n";
  out << startLine << "Number of clauses: " << m_clauses.size() << "\n";
  out << startLine << "Number of binary clauses: " << nbBin << "\n";
  out << startLine << "Number of ternary clauses: " << nbTer << "\n";
  out << startLine << "Number of clauses larger than 3: " << nbMoreThree << "\n";
  out << startLine << "Number of literals: " << nbLits << "\n";
} // displaystat

}
