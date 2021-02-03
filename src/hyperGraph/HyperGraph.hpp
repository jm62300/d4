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
#include <cassert>

namespace d4
{
/**
   We use a raw representation of the hyper graph.
   -> [size1] [...elts1 ...] [size2] [... elts2...] .......
*/
class HyperGraph
{
 private:
  unsigned *m_hypergraph;
  unsigned m_hypergraphCapacity;
  unsigned m_hypergraphSize;

 public:
  HyperGraph();
  HyperGraph(unsigned capacity);
  ~HyperGraph();

  void displayHyperGraph();
  void init(unsigned capacity);

  inline void incSize(){m_hypergraphSize++;}
  inline void decSize(){m_hypergraphSize--;}
  inline void setSize(unsigned size){m_hypergraphSize = size;}
  inline unsigned *getEdges(){return m_hypergraph;}
  inline unsigned getSize(){return m_hypergraphSize;}

  inline unsigned operator [] (unsigned i) const
  {
    assert(i<m_hypergraphCapacity);
    return m_hypergraph[i];
  }
  
  inline unsigned &operator [] (unsigned i)
  {
    assert(i<m_hypergraphCapacity);
    return m_hypergraph[i];
  }
};
} // d4
