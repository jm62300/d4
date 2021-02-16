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

#include <vector>
#include <boost/program_options.hpp>

#include "src/problem/ProblemTypes.hpp"

namespace d4
{
namespace po = boost::program_options;

class PhaseSelectorManager
{
 protected:
  std::vector<unsigned> m_bucketNumber;

  PhaseSelectorManager(std::vector<unsigned> &bucketNumber);
  
 public:
  virtual ~PhaseSelectorManager(){}
  
  static PhaseSelectorManager *makePhaseSelectorManager(
      po::variables_map &vm,
      std::vector<unsigned> &bucketNumber);
  
  virtual bool isStillOk(std::vector<Var> &component) = 0;
};
} // d4
