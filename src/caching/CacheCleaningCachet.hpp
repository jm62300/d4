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
#include "CacheCleaningManager.hpp"
#include "CachedBucket.hpp"

namespace d4 {
template <class T> class CacheCleaningManager;
template <class T> class Cache;
template <class T> class CacheCleaningCachet : public CacheCleaningManager<T> {
private:
  unsigned long m_limitNbEntry;
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
     @param[in] limit, the limit of entry before we clean the cache.
   */
  CacheCleaningCachet(Cache<T> *cache, bool smudge, unsigned long limit) {
    m_nbReduceCall = 0;
    m_nbRemoveEntry = 0;
    m_limitNbEntry = limit;
    m_smudge = smudge;
    this->m_cache = cache;
  } // constructor

  /**
     We init the count of a bucket with the number of times we ask for an entry
     in the cache.

     @param[out] cb, the cached bucket we want to init.
   */
  void initCountCachedBucket(CachedBucket<T> *cb) {
    cb->reinitCount(m_cache->getNbNegativeHit());
  } // initCountCachedBucket

  /**
     For the cachet strategy we do not update information about the cached
     bucket since we use the age of the bucket to know if we should remove it.

     @param[out] cb, the cached bucket we want to init.
     @param[in] nbVar, not used here.
   */
  void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar) {
    // do nothing.
  } // updateCountCachedBucket

  /**
     Remove 75% of the cache entries once we have reached a predifined limit of
     entries.
   */
  void reduceCache() {
    if (m_cache->getNbEntry() < m_limitNbEntry)
      return;
    m_nbReduceCall++;

    // get the limit by sorting the element.
    auto &hashTable = m_cache->getHashTable();
    int limit =
        (m_cache->getNbNegativeHit() >> 1) + (m_cache->getNbNegativeHit() >> 2);

    for (unsigned i = 0; i < hashTable.size(); i++) {
      std::vector<CachedBucket<T>> &v = hashTable[i];
      for (unsigned j = 0; j < v.size();) {
        CachedBucket<T> &cb = v[j];
        if (!cb.smudge() && cb.count() < limit) {
          this->releaseMemory(cb.data, cb.szData(), i, m_smudge);
          if (m_smudge) {
            cb.smudge(true);
            j++;
          } else {
            v[j] = v.back();
            v.pop_back();
          }

          m_cache->decrementNbEntry();
          m_nbRemoveEntry++;
        } else {
          cb.divCount();
          j++;
        }
      }
    }

    std::cout << "c Number of entries removed: " << m_nbRemoveEntry << "/"
              << m_cache->getNbEntry() << "\n";
  } // reduceCache

  /**
     We delete cb, but it is useful right now.

     @param[in] cb, the bucket used.
  */
  void wrongSmudge(CachedBucket<T> &cb) {
    // nothing to do.
  } // wrongSmudge

  /**
     Print out statistics about the cleaning process.

     @param[in] out, the stream where are print out the information.
   */
  void printCleaningInfo(std::ostream &out) {
    out << "c Number of reduce calls: " << m_nbReduceCall << "\n";
    out << "c Number of entry removedreduce: " << m_nbRemoveEntry << "\n";
  }
};

} // namespace d4
