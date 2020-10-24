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

#include "MethodManager.hpp"
#include "ModelCounter.hpp"

namespace d4
{

/**
   Consider the option in order to generate an instance of the wanted method.

   @param[in] vm, the map of option.
 */
MethodManager *MethodManager::makeMethodManager(po::variables_map &vm)
{
  std::string meth = vm["method"].as<std::string>();

  if(meth == "counting") return new ModelCounter<int>(vm);
  return NULL;
} // makeMethodManager

} // d4
