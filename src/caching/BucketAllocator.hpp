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

#include <cassert>
#include <vector>
#include <deque>
#include <functional>

namespace d4
{
struct Released
{
  char *data;
  int posInHash;

  Released(char *_data, int _posInHash) : data(_data), posInHash(_posInHash){}
};

class BucketAllocator
{
 private:
  std::vector<char *> m_allocateData;
  char *m_data;
  unsigned long m_sizeFirstPage;
  unsigned long m_sizeAdditionalPage;
  unsigned long m_sizeData;
  unsigned long m_posInData;

  // freespace[i][j] points to a free memory space of size i
  std::vector<std::deque<Released>> m_freeSpace;
  unsigned long int m_allMemory;
  unsigned long int m_freeMemory;
  unsigned long int m_pageData;
  unsigned long int m_usedMemory;
  bool isInit = false;
  
  std::function<void(char *, int)> m_removeSmudgeEntry;
  
 public:
  ~BucketAllocator();

  inline unsigned long int usedMemory(){return m_usedMemory;}
  
  inline double remainingMemory()
  {
    return ((double) m_freeMemory + (m_sizeData - m_posInData)) / (double) m_allMemory;
  } // remainingMemory


  void init(unsigned long sizeFirstPage,
            unsigned long sizeAdditionalPage,
            std::function<void(char *, int)> removeSmudgeEntry);

  char *getArray(unsigned size);
  
  void releaseMemory(char *m, unsigned size, int posInHash = -1);
};

} // d4
