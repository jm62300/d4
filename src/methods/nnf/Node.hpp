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

#include "src/exceptions/NodeException.hpp"
#include "src/problem/ProblemManager.hpp"

template <class T, typename U> class DecomposableAndNode;
template <class T, typename U> class BinaryDeterministicOrNode;
template <class T, typename U> class UnaryNode;
template <class T> class TrueNode;
template <class T> class FalseNode;

namespace d4
{
enum TypeNode {TypeIteNode, TypeUnaryNode,
               TypeDecAndNode, TypeTrueNode, TypeFalseNode};
enum ValueVar {isTrue, isFalse, isNotAssigned};

template <class T> class Node
{
 public:
  struct
  {
    unsigned typeNode:4;
    unsigned stamp:28;
  } header;
  
  
  Node()
  {
    header.typeNode = 0;
    header.stamp = 0;
  }

  template <typename U>
  inline T computeNbModels(std::vector<ValueVar> &fixedValue,
                           ProblemManager &problem,
                           unsigned globalStamp)
  {
    switch(header.typeNode)
    {
      case TypeIteNode:
        return BinaryDeterministicOrNode<T,U>::computeNbModels(fixedValue, problem, globalStamp);
      case TypeDecAndNode:
        return DecomposableAndNode<T,U>::computeNbModels(fixedValue, problem, globalStamp);
      case TypeUnaryNode:
        return UnaryNode<T,U>::computeNbModels(fixedValue, problem, globalStamp);
      case TypeTrueNode:
        return TrueNode<T>::computeNbModels(fixedValue, problem, globalStamp);
      case TypeFalseNode:
        return FalseNode<T>::computeNbModels(fixedValue, problem, globalStamp);
      default:
        throw (NodeException("Node type unknown",__FILE__, __LINE__));
    }
  } // computeNbModels
};
} // d4
