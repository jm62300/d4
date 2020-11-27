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

#include <iostream>
#include <vector>

#include "src/problem/ProblemManager.hpp"

#include "Node.hpp"

namespace d4
{
template <class T> class DataBranch
{
 public:
  Node<T> *d;
  std::vector<Lit> unitLits;
  std::vector<Var> freeVars;  

  inline unsigned sumFreeUnit(){return unitLits.size() + freeVars.size();}
};

template <class T, typename U> class Branch
{

 private:
  /**
     Ask if l is unsatisfiable if the variable is assigned to some given value.
     
     @param[in] value, the value we assign the variable (l>>1)
     @param[in] l, the literal we want to know if it unsatisfiable or not.

     \return true if l is unsat under the value, false otherwise.
  */
  inline bool isUnsatLit(ValueVar value, U l)
  {
    if(value == ValueVar::isNotAssigned) return false;    
    return (value == ValueVar::isTrue && (l&1))
        || (value == ValueVar::isFalse && !(l&1));
  } // isUnsatLit
  
  
 public:
  
  U nbUnits;
  U nbFree;
  Node<T> *d;

  /**
     Regarding a branch, ask for the number of models of the formula under an
     interpretation.

     @param[in] func, give for the type of node the deallocate function.
     @param[in] b, the branch we consider.
     @param[in] data, the place to get the data.
     @param[in] fixedValue, the assigment we consider.
     @param[in] problem, the problem we are solving (use to get information about weight).

     \return the number of models.
  */
  T computeNbModels(T (**func)(),
                    U *data,
                    std::vector<ValueVar> &fixedValue,
                    ProblemManager &problem,
                    unsigned globalStamp)
  {
    T computeWeight = 1;

    for(unsigned i = 0 ; i<nbUnits ; i++)
    {
      U l = data[i];
      if(isUnsatLit(fixedValue[l>>1], l)) return 0;
      computeWeight *= T(problem.getWeightLit()[l]);
    }
    
    T c = reinterpret_cast<T (**)(Node<T> *,
                                  T (**func)(),
                                  std::vector<ValueVar> &,
                                  ProblemManager &,
                                  unsigned)>
          (func)[d->header.typeNode](d, func, fixedValue, problem, globalStamp);

    for(unsigned i = 0 ; i<nbFree ; i++)
    {
      U v = data[nbUnits + i];
      switch(fixedValue[v])
      {
        case isFalse:
          computeWeight *= T(problem.getWeightLit()[(v<<1) | 1]);
          break;
        case isTrue:
          computeWeight *= T(problem.getWeightLit()[v<<1]);
          break;
        default:
          computeWeight *= T(problem.getWeightVar()[v]);
      }
    }
    
    return c * computeWeight;
  } // computeNbModels
};
} // d4
