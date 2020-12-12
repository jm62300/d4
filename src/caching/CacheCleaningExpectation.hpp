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
template<class T> class CacheCleaningExpectation : public CacheCleaningManager<T>
{
 private:
  bool m_smudge;
  unsigned m_nbReduceCall;
  unsigned long m_nbRemoveEntry;
  unsigned long m_nbFailedInCache;
  unsigned long m_limitNegativeHit;
  int m_nbVar;

  std::vector<double> m_ratio;
  std::vector<unsigned long> m_sizeVarCacheHit;
  std::vector<unsigned long> m_nbCacheWithSizeVar;
  std::vector<unsigned> m_wrongSmudge;

  using CacheCleaningManager<T>::m_cache;
  
 public:
  
  /**
     Constructor.

     @param[in] cache, the cache where is applied the cleaning process.     
     @param[in] smudge, control if we directly remove the entries or if we
     postpone until we really need the memory.
     @param[in] nbVar, the number of variables in the problem.     
     @param[in] limitNegativehit, the number of negative hits before calling the
     reduction.
     @param[in] ratio, the limit ratio.
   */
  CacheCleaningExpectation(Cache<T> *cache, bool smudge, int nbVar,
                           unsigned long limitNegativehit,
                           double ratio)
  {
    m_limitNegativeHit = limitNegativehit;
    m_nbVar = nbVar;
    m_nbFailedInCache = 0;
    m_nbReduceCall = 0;
    m_nbRemoveEntry = 0;
    m_smudge = smudge;
    this->m_cache = cache;

    m_ratio.resize(nbVar + 1, ratio);
    m_wrongSmudge.resize(nbVar + 1, 0);
    m_sizeVarCacheHit.resize(nbVar + 1, 0);
    m_nbCacheWithSizeVar.resize(nbVar + 1, 0);
  } // constructor


  /**
     We init the count of a bucket with the number of times we ask for an entry
     in the cache.

     @param[out] cb, the cached bucket we want to init.
   */
  void initCountCachedBucket(CachedBucket<T> *cb)
  {
    assert(cb);
    cb->reinitCount(cb->nbVar());
    m_nbCacheWithSizeVar[cb->nbVar()]++;
  } // initCountCachedBucket

  
  /**
     For the cachet strategy we reinit the score of the bucket, that means newly
     possitively hit buckets get the priority.

     @param[in] cb, the cached bucket we want to init.
     
     @param[in] nbVar, a number of variables (because cb can be NULL the number
     of variables cannot be related to cb).
   */
  void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar)
  {
    if(cb)
    {
      m_sizeVarCacheHit[cb->nbVar()]++;
      cb->incCount(1);
      cb->setTrueDirty();
    }
    else
    {      
      m_nbFailedInCache++;
    }
  } // updateCountCachedBucket

  
  /**
     We remove the entry regarding if they have been used recently and depending
     their number of variables.
   */
  void reduceCache()
  {
    if(m_nbFailedInCache < m_limitNegativeHit) return;
    m_nbFailedInCache = 0;
    m_nbReduceCall++;
#if 1
    for(int i = 0 ; i<m_nbVar ; i++)
    {
      if(m_wrongSmudge[i]) m_ratio[i] *= 0.99; else m_ratio[i] *= 1.01;      
      // std::cout << i << "(" << m_wrongSmudge[i] << "/" << m_ratio[i]<< ") ";
      m_wrongSmudge[i] >>= 1;      
    }
#endif
    // std::cout << "\n";

    auto &hashTable = m_cache->getHashTable();
    for(unsigned i = 0 ; i<hashTable.size() ; i++)
    {
      std::vector< CachedBucket<T> > &v = hashTable[i];

      for(unsigned j = 0 ; j<v.size() ; )
      {
        CachedBucket<T> &cb = v[j];
        if(cb.smudge()){j++; continue;}
        
        double ratio = (double) m_sizeVarCacheHit[cb.nbVar()] / (double) m_nbCacheWithSizeVar[cb.nbVar()];
        bool mustBeKept = (cb.count() || cb.dirty() || ratio > m_ratio[cb.nbVar()]);

        if(mustBeKept)
        {
          cb.divCount();
          if(!cb.count() && cb.dirty())
          {
            cb.setFalseDirty();
            if(m_sizeVarCacheHit[cb.nbVar()]) m_sizeVarCacheHit[cb.nbVar()]--;
          }
          cb.setFalseDirty();          
          j++;
        } else
        {
          if(m_sizeVarCacheHit[cb.nbVar()]) m_sizeVarCacheHit[cb.nbVar()]--;
          assert(m_nbCacheWithSizeVar[cb.nbVar()]);
          m_nbCacheWithSizeVar[cb.nbVar()]--;

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
          
          m_nbRemoveEntry++;
          m_cache->decrementNbEntry();
        }
      }
    }

    std::cout << "c Number of entries removed: " << m_nbRemoveEntry
              << "/" << m_cache->getNbEntry() << "\n";
  } //reduceCache


  /**
     We delete cb, but it is useful right now.

     @param[in] cb, the bucket used.
   */
  void wrongSmudge(CachedBucket<T> &cb)
  {
    m_wrongSmudge[cb.nbVar()]++;
  } // wrongSmudge

  
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
