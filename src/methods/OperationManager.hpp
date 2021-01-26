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
#include <boost/multiprecision/gmp.hpp>
#include <boost/program_options.hpp>

#include "DataBranch.hpp"
#include "CountingOperation.hpp"

namespace d4
{
namespace po = boost::program_options;
template <class T> class Operation
{
 public:
  virtual ~Operation() {}

  virtual T createTop() = 0;
  virtual T createBottom() = 0;
  virtual void manageResult(T &result, po::variables_map &vm,
                            std::ostream &out) = 0;
  virtual T manageBottom() = 0;
  virtual T manageTop(std::vector<Var> &component) = 0;
  virtual T manageBranch(DataBranch<T> &e) = 0;
  virtual T manageDeterministOr(DataBranch<T> *elts, unsigned size) = 0;
  virtual T manageDecomposableAnd(T *elts, unsigned size) = 0;
};
} // d4
