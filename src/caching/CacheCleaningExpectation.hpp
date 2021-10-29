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

struct StatVarSizeCache {
  unsigned long negative;
  unsigned long positive;
  unsigned long number;
};

template <class T>
class CacheCleaningExpectation : public CacheCleaningManager<T> {
private:
  unsigned m_nbReduceCall;
  unsigned long m_nbRemoveEntry;
  unsigned long m_nbPositiveHit;
  unsigned long m_nbNegativeHit;
  unsigned long m_limitNegativeHit;
  int m_nbVar;

  std::vector<StatVarSizeCache> m_statVar;
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
  CacheCleaningExpectation(Cache<T> *cache, int nbVar,
                           unsigned long limitNegativehit, double ratio) {
    m_limitNegativeHit = limitNegativehit;
    m_nbVar = nbVar;
    m_nbNegativeHit = 0;
    m_nbPositiveHit = 0;
    m_nbReduceCall = 0;
    m_nbRemoveEntry = 0;
    this->m_cache = cache;

    m_statVar.resize(nbVar + 1, {0, 0, 0});
  } // constructor

  /**
     We init the count of a bucket with the number of times we ask for an entry
     in the cache.

     @param[out] cb, the cached bucket we want to init.
   */
  void initCountCachedBucket(CachedBucket<T> *cb) {
    m_statVar[cb->nbVar()].number++;
  } // initCountCachedBucket

  /**
   * @brief Update the information about the bucket.
   *
   * @param cb is the cached bucket we want to init.
   * @param nbVar a number of variables (because cb can be NULL the number of
   * variables cannot be related to cb).
   */
  void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar) {
    if (cb) {
      m_statVar[nbVar].positive++;
      m_nbPositiveHit++;
    } else {
      m_statVar[nbVar].negative++;
      m_nbNegativeHit++;
    }
  } // updateCountCachedBucket

  /**
   * @brief We remove the entry regarding if they have been used recently and
   * depending their number of variables.
   */
  void reduceCache() {
    m_nbReduceCall++;

    for (int i = 0; i < m_nbVar; i++) {
      if (m_statVar[i].positive || m_statVar[i].negative) {
        std::cout << i << " " << m_statVar[i].positive << " "
                  << m_statVar[i].negative << "\n";
      }
    }

    auto &hashTable = m_cache->getHashTable();
    for (unsigned i = 0; i < hashTable.size(); i++) {
      // TODO:
    }

    std::cout << "Please find something really smart to manage the cache "
                 "because your previous ideas was shit!\n";
    assert(0);

    std::cout << "c Number of entries removed: " << m_nbRemoveEntry << "/"
              << m_cache->getNbEntry() << "\n";
  } // reduceCache

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
