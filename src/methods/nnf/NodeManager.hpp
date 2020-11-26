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

#include <bitset> 
#include <vector>

#include "Node.hpp"
#include "TrueNode.hpp"
#include "FalseNode.hpp"
#include "BinaryDeterministicOrNode.hpp"
#include "Branch.hpp"

namespace d4
{
template <class T> class NodeManager;
template <class T, typename U> class NodeManagerTyped : public NodeManager<T>
{
  using NodeManager<T>::m_memoryPages;
  using NodeManager<T>::m_posInMemoryPage;
  using NodeManager<T>::m_data;
  using NodeManager<T>::PAGE_SIZE;
  
 public:
  /**
     The constructor allocate the first memory page.
   */
  NodeManagerTyped()
  {
    m_data = new uint8_t[PAGE_SIZE];
    m_memoryPages.push_back(m_data);
    m_posInMemoryPage = 0;
  } // NodeManagerTyped
  
  /**
     Create a binary deterministic OR node.

     @param[in] left, the left branch.
     @param[in] right, the right branch.

     \return a BinaryDeterministicOrNode that make the disjcution between left
     and right.
  */
  Node<T> *makeBinaryDeterministicOrNode(DataBranch<T> &left, DataBranch<T> &right)
  {
    unsigned memoryNeeded = sizeof(BinaryDeterministicOrNode<T,U>)
                            + left.sumFreeUnit() + right.sumFreeUnit(); // in number of bytes.

    uint8_t *data = NodeManager<T>::getMemory(memoryNeeded);
    BinaryDeterministicOrNode<T,U> *ret =
        reinterpret_cast<BinaryDeterministicOrNode<T,U> *>(data);
    ret->init(left, right);
    
    return ret;
  } // makeBinaryDeterministicOrNode

};

template <class T> class NodeManager
{
 protected:
  std::vector<uint8_t *> m_memoryPages;
  unsigned m_posInMemoryPage;
  uint8_t *m_data;

  FalseNode<T> falseNode;
  TrueNode<T> trueNode;

  /**
     'Allocate' an ammount of bytes.

     @param[in] nbBytes, the ammount of memory we want to allocate.

     \return a pointer on the memory we allocate.
   */
  inline uint8_t *getMemory(unsigned nbBytes)
  {
    assert((nbBytes<<2) < PAGE_SIZE); // we check out that the PAGE is large enough.
    if(m_posInMemoryPage + nbBytes >= PAGE_SIZE)
    {
      m_posInMemoryPage = 0;
      m_data = new uint8_t[PAGE_SIZE];
      m_memoryPages.push_back(m_data);
    }
    
    m_posInMemoryPage += nbBytes;
    return &m_data[m_posInMemoryPage - nbBytes];
  } // getMemory
  
 public:
  static const unsigned PAGE_SIZE = 1<<20;
  
  /**
     The destructor free the memory.
   */
  virtual ~NodeManager()
  {
    for(auto &p : m_memoryPages) delete[] p;
    m_posInMemoryPage = 0;
  } // destructor  
  
  /**
     Node constructor factory.

     @param[in] nbVar, the number of variables in the problem.
   */
  static NodeManager<T> *makeNodeManager(unsigned nbVar)
  {
    if(nbVar < (1<<8)) return new NodeManagerTyped<T, uint8_t>();
    if(nbVar < (1<<16)) return new NodeManagerTyped<T, uint16_t>();
    return new NodeManagerTyped<T, uint32_t>();
  } // makeNodeManager
  
  inline Node<T> *makeTrueNode(){return &trueNode;}
  inline Node<T> *makeFalseNode(){return &falseNode;}

  virtual Node<T> *makeBinaryDeterministicOrNode(DataBranch<T> &left,
                                                 DataBranch<T> &right) = 0;
  
};
} // d4
