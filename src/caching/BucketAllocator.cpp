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


#include "BucketAllocator.hpp"


namespace d4
{
/**
   Initialize the data structure regarding the configuration (ie. number of
   variables, maximum number of clauses and the lenght of the largest clause).

   @param[in] sizeFirstPage, the amount of bytes for the first page.
   @param[in] sizeAdditionalPage, the amount of bytes for the additional pages.
   @param[in] removeSmudgeEntry, function pointer used to delete the smudge entry.
*/
void BucketAllocator::init(unsigned long sizeFirstPage,
          unsigned long sizeAdditionalPage,
          std::function<void(char *, int)> removeSmudgeEntry)
{
  if(isInit) return;
  isInit = true;
    
  m_removeSmudgeEntry = removeSmudgeEntry;
  m_allMemory = m_freeMemory = m_posInData = 0;
  m_sizeFirstPage = sizeFirstPage;
  m_sizeAdditionalPage = sizeAdditionalPage;    
  m_sizeData = m_sizeFirstPage;

  // we cannot reinit ... at least for the moment
  assert(!m_allocateData.size());
  m_data = new char[m_sizeData];
  m_allocateData.push_back(m_data);
  m_allMemory += m_sizeData;
  m_usedMemory = 0;
} // init


/**
   Get a pointer on an available array where we can store the data we want to
   save into the bucket.

   @param[in] size, the size of the entry we want.

   \return a pointer on a memory block.
*/
char *BucketAllocator::getArray(unsigned size)
{
  m_usedMemory += size;
  char *ret = NULL;    

  if(size < m_freeSpace.size() && m_freeSpace[size].size())
  {
    Released &r = m_freeSpace[size].back();      
    ret = r.data;

    if(r.posInHash != -1) m_removeSmudgeEntry(r.data, r.posInHash);

    m_freeSpace[size].pop_back();
    m_freeMemory -= size;
    return ret;
  }
    
  // take a fresh entry
  if(m_posInData + size > m_sizeData)
  {
    unsigned rSz = m_sizeData - m_posInData;
    if(m_freeSpace.size() <= rSz) m_freeSpace.resize(rSz + 1, std::deque<Released>());
    m_freeSpace[rSz].push_back(Released(&m_data[m_posInData], -1));
    m_freeMemory += rSz;

    printf("c Allocate a new page for the cache %lu\n", m_freeMemory);

    m_sizeData = m_sizeAdditionalPage;
    m_posInData = 0;
    m_data = new char[m_sizeData];
    m_allocateData.push_back(m_data);

    m_allMemory += m_sizeData;
  }
    
  ret = &m_data[m_posInData];
  m_posInData += size;
  return ret;
} // getArray


/**
   Release some memory of a given size and store this information in
   freespace.

   @param[in] m, the memory we want to release
   @param[in] size, the size of the memory block
   @param[in] posInHash, a position in the hash table (in the vector).
*/
void BucketAllocator::releaseMemory(char *m, unsigned size, int posInHash)
{
  m_usedMemory -= size;
    
  if((m_posInData - size) > 0 && &m_data[m_posInData - size] == m)
    m_posInData -= size;
  else
  {
    if(size >= m_freeSpace.size()) m_freeSpace.resize(size + 1, std::deque<Released>());
    m_freeSpace[size].push_front(Released(m, posInHash));
    m_freeMemory += size;
  }
}// reverseLastBucket


} // d4
