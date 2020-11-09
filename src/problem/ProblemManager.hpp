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

#include <boost/program_options.hpp>

namespace d4
{
namespace po = boost::program_options;
class ProblemManager
{
 protected:
  int m_nbVar;
  
 public:
  static ProblemManager *makeProblemManager(po::variables_map &vm,
                                            std::ostream &out);
  
  virtual ~ProblemManager(){;}
  unsigned getNbVar(){return m_nbVar;}
  void setNbVar(int n){m_nbVar = n;}
  
  virtual void display(std::ostream &out) = 0;
  virtual void displayStat(std::ostream &out, std::string startLine) = 0;
};
}
