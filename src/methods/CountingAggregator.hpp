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

#include "DataBranch.hpp"

namespace d4
{
template<class T, class U> class Aggregator;
template <class T> class CountingAggregator : public Aggregator<T, T>
{
 private:
  ProblemManager *m_problem;
  
 public:

  CountingAggregator() = delete;
  
  /**
     Constructor.

     @param[in] problem, allows to get information about the problem such as
     weights.
   */
  CountingAggregator(ProblemManager *problem) : m_problem(problem)
  {
    
  } // constructor.

  
  /**
     Compute the sum of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
  */
  T aggregateDeterministOr(DataBranch<T> *elts, unsigned size)
  {
    T ret = 0;
    for(unsigned i = 0 ; i<size ; i++)
    {
      ret = ret + (elts[i].d * m_problem->computeWeightUnitFree<T>(
          elts[i].unitLits, elts[i].freeVars));
    }
    
    return ret;
  } // aggregateDeterministOr

  
  /**
     Compute the product of the given elements.

     @param[in] elts, the elements we want to get the product.
     @param[in] size, the number of elements.

     \return the product of each element of elts.
   */
  T aggregateDecomposableAnd(T *elts, unsigned size)
  {
    T ret = 1;
    for(unsigned i = 0 ; i<size ; i++) ret = ret * elts[i];
    return ret;
  } // aggregateDecomposableAnd
};

} // d4


