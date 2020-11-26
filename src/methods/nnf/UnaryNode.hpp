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
#include "Branch.hpp"

namespace d4
{
template <class T, typename U> class UnaryNode : public Node<T>
{
 public:
  using Node<T>::header;
  
  UnaryNode() = delete;
  
  T nbModels;
  Branch<T, U> b;
  
  // data[0 .. b.nbLit - 1] gives the unit literals for the left branch.
  // data[b.nbLit .. b.nbLit + b.nbFree - 1] gives the free variables for the left branch.
  U data[0];

  
  /**
     Init the two branches using the data coming from the solver.

     @param[in] left, the left branch.
     @param[in] right, the right branch.
   */
  void init(DataBranch<T> &branch)
  {
    header.typeNode = TypeNode::TypeUnaryNode;
    b.d = branch.d;

    b.nbUnits = branch.unitLits.size();
    b.nbFree = branch.freeVars.size();    

    unsigned pos = 0;
    for(auto &l : branch.unitLits) data[pos++] = l.intern();
    for(auto &v : branch.freeVars) data[pos++] = v;
  } // init
  
  /**
     Ask for the number of models of the formula under an interpretation.

     @param[in] fixedValue, the assigment we consider
     @param[in] problem, the problem we are solving (use to get information about weight).

     \return the number of models.
   */
  T computeNbModels(std::vector<ValueVar> &fixedValue,
                    ProblemManager &problem,
                    unsigned globalStamp)
  {
    if(header.stamp == globalStamp) return nbModels;
    nbModels = b.computeNbModels(data, fixedValue, problem, globalStamp);
    header.stamp = globalStamp;
    return nbModels;
  } // computeNbModels


  /**
     Ask if the formula is satisfiable under an interpretation.

     @param[in] fixedValue, the assigment we consider

     \return true if the problem is satisfiable, falst otherwise.
   */
  bool isSAT(std::vector<ValueVar> &fixedValue)
  {
    // TODO
    return true;
  } // isSAT


  /**
     Print out the NNF in a stream.

     @param[in] out, the stream where we print out the formula.
     @param[in] certif, boolean that control if we certify the formula.
   */ 
  void printNNF(std::ostream& out)
  {
    // TODO: d->printNNF(out);
  } // printNNF
};
} // d4
