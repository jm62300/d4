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
#include "ScoringMethod.hpp"


#include "cnf/Mom.hpp"

namespace d4
{

/**
   Select from the arguments store in vm the good scoring method and return it.

   @param[in] vm, the arguments on the command line.
   @pararm[in] p, the problem manager.

   \return the scoring method
 */
ScoringMethod *ScoringMethod::makeScoringMethod(po::variables_map &vm, SpecManager &p)
{
  std::string in = vm["input"].as<std::string>();
  std::string extension = in.substr(in.find_last_of(".") + 1);

  if(extension == "cnf" || extension == "dimacs")
  {
    std::string meth = vm["scoring-method"].as<std::string>();

    try
    {
      SpecManagerCnf &ps = dynamic_cast<SpecManagerCnf&>(p);

      if(meth == "mom") return new Mom(ps);
      return NULL;
    
    }
    catch (std::bad_cast& bc)
    {
      std::cerr << "bad_cast caught: " << bc.what() << '\n';
      std::cerr << "A CNF formula was expeted\n";
    }    
  }
  
  return NULL;
} // makeScoringMethod
}
