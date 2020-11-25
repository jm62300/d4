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

#include <boost/multiprecision/gmp.hpp>

#include "MethodManager.hpp"
#include "ModelCounter.hpp"
#include "DDnnfCompiler.hpp"

namespace d4
{

/**
   Consider the option in order to generate an instance of the wanted method.

   @param[in] vm, the map of option.
 */
MethodManager *MethodManager::makeMethodManager(po::variables_map &vm,
                                                std::ostream &out)
{
  std::string meth = vm["method"].as<std::string>();
  int precision = vm["float-precision"].as<int>();

  boost::multiprecision::mpf_float::default_precision(precision); // we set the precision
  if(meth == "counting")
  {
    bool isFloat = vm["float"].as<bool>();
    if(!isFloat) return new ModelCounter<boost::multiprecision::mpz_int>(vm);
    else return new ModelCounter<boost::multiprecision::mpf_float>(vm);
  }

  if(meth == "ddnnf-compiler")
  {
    bool isFloat = vm["float"].as<bool>();
    if(!isFloat) return new DDnnfCompiler<boost::multiprecision::mpz_int>(vm);
    else return new DDnnfCompiler<boost::multiprecision::mpf_float>(vm);
  }

  throw (FactoryException("Cannot create a MethodManager",__FILE__, __LINE__));
} // makeMethodManager

} // d4
