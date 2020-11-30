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

#include "BucketManager.hpp"
#include "CachedBucket.hpp"
#include "CacheCleaningManager.hpp"

namespace d4
{
template<class T> class CacheCleaningManager;
template<class T> class Cache;
template<class T> class CacheCleaningSharpSAT : public CacheCleaningManager<T>
{
 private:
  bool m_smudge;
  unsigned m_nbReduceCall;
  unsigned long m_nbRemoveEntry;

  using CacheCleaningManager<T>::m_cache;
  
 public:
  
  /**
     Constructor.

     @param[in] cache, the cache where is applied the cleaning process.     
     @param[in] smudge, control if we directly remove the entries or if we
     postpone until we really need the memory.
   */
  CacheCleaningSharpSAT(Cache<T> *cache, bool smudge)
  {
    m_nbReduceCall = 0;
    m_nbRemoveEntry = 0;
    m_smudge = smudge;
    this->m_cache = cache;
  } // constructor


  /**
     We init the count of a bucket with the number of times we ask for an entry
     in the cache.

     @param[out] cb, the cached bucket we want to init.
   */
  void initCountCachedBucket(CachedBucket<T> *cb)
  {
    assert(cb);
    cb->reinitCount(m_cache->getNbNegativeHit());
  } // initCountCachedBucket

  
  /**
     For the cachet strategy we reinit the score of the bucket, that means newly
     possitively hit buckets get the priority.

     @param[out] cb, the cached bucket we want to init.
     @param[in] nbVar, not used here.
   */
  void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar)
  {
    if(cb) cb->reinitCount(m_cache->getNbNegativeHit());
  } // updateCountCachedBucket

  
  /**
     Reduce half the cache once we reach 90% of the memory capacity.
   */
  void reduceCache()
  {
    if(m_cache->getBucketManager()->remainingMemory() > 0.1) return;    
    m_nbReduceCall++;

    // get the limit by sorting the element.
    auto &hashTable = m_cache->getHashTable();
    std::vector<int> vecCount;

    for(auto &v : hashTable)
      for(auto b : v) if(!b.smudge()) vecCount.push_back(b.count());

    if(!vecCount.size()) return;
    std::sort(vecCount.begin(), vecCount.end());
    int limit = vecCount[vecCount.size() >> 1];

    // clean the cache.
    for(unsigned i = 0 ; i<hashTable.size() ; i++)
    {
      std::vector< CachedBucket<T> > &v = hashTable[i];

      for(unsigned j = 0 ; j<v.size() ; )
      {
        CachedBucket<T> &cb = v[j];
        if(!cb.smudge() && cb.count() < limit)
        {
          this->releaseMemory(cb.data, cb.szData(), i, m_smudge);
          if(m_smudge)
          {
            cb.smudge(true);
            j++;
          }
          else            
          {
            v[j] = v.back();
            v.pop_back();
          } 
          
          m_cache->decrementNbEntry();
          m_nbRemoveEntry++;
        } else
        {
          cb.divCount();
          j++;
        }
      }
    }

    std::cout << "c Number of entries removed: " << m_nbRemoveEntry
              << "/" << m_cache->getNbEntry() << "\n";
  } //reduceCache


  /**
     Print out statistics about the cleaning process.

     @param[in] out, the stream where are print out the information.
   */
  void printCleaningInfo(std::ostream &out)
  {
    out << "c Number of reduce calls: " << m_nbReduceCall << "\n";
    out << "c Number of entry removedreduce: " << m_nbRemoveEntry << "\n";
  }
};
  
} // d4
