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
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#pragma once

#include <vector>

#include "CacheManager.hpp"
#include "CachedBucket.hpp"
#include "cleaning/CacheCleaningManager.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/options/cache/OptionCacheManager.hpp"

namespace d4 {

template <class T>
class CacheList : public CacheManager<T> {
 private:
  static constexpr uint64_t SIZE_HASH = 2097152;
  static constexpr uint32_t END_OF_LIST = 0xFFFFFFFF;  // "null" index

  struct CollisionNode {
    CachedBucket<T> bucket;
    uint32_t next;
  };

  std::vector<uint32_t> hashTable;
  std::vector<CollisionNode> nodePool;
  uint32_t freeListHead;

 public:
  /**
   * @brief Construct a new Cache List object
   */
  CacheList(const OptionCacheManager& options, unsigned nbVar,
            FormulaManager* specs, std::ostream& out)
      : CacheManager<T>(options, nbVar, specs, out) {
    out << "c [CACHE LIST CONSTRUCTOR]\n";
    initHashTable(nbVar);
  }

  /**
   * @brief Destroy the Cache List object
   */
  ~CacheList() {
    hashTable.clear();
    nodePool.clear();
  }

  /**
   * @brief Add an entry in the cache.
   */
  inline void pushInHashTable(CachedBucket<T>& cb, uint64_t hashValue, T val) {
    uint64_t index = hashValue & (SIZE_HASH - 1);
    uint32_t newNodeIdx;

    // Reuse a deleted slot if available, otherwise grow the pool
    if (freeListHead != END_OF_LIST) {
      newNodeIdx = freeListHead;
      freeListHead = nodePool[freeListHead].next;
      nodePool[newNodeIdx].bucket = cb;
    } else {
      newNodeIdx = nodePool.size();
      nodePool.push_back({cb, END_OF_LIST});
    }

    // Insert at the head of the collision list
    nodePool[newNodeIdx].next = hashTable[index];
    hashTable[index] = newNodeIdx;

    // Update statistics and lock bucket
    CachedBucket<T>& cbIn = nodePool[newNodeIdx].bucket;
    cbIn.lockedBucket(val);

    this->m_nbCreationBucket++;
    this->m_sumDataSize += cbIn.dataBucket.szData();
    this->m_cacheCleaningManager->initCountCachedBucket(cbIn);
    this->m_nbEntry++;
  }

  /**
   * @brief Research in the set of buckets if the bucket already exists.
   */
  CachedBucket<T>* bucketAlreadyExist(DataBucket& db,
                                      uint64_t hashValue) override {
    char* refData = db.data;
    uint64_t index = hashValue & (SIZE_HASH - 1);  // Fast bitwise modulo

    uint32_t curr = hashTable[index];

    // Traverse the index-based linked list
    while (curr != END_OF_LIST) {
      CachedBucket<T>& cbi = nodePool[curr].bucket;

      if (db.sameHeader(cbi.dataBucket) &&
          !memcmp(refData, cbi.dataBucket.data, cbi.dataBucket.szData())) {
        this->m_nbPositiveHit++;
        return &cbi;
      }
      curr = nodePool[curr].next;  // Move to next node
    }

    this->m_nbNegativeHit++;
    return NULL;
  }

  /**
   * Create a bucket and store it in the cache.
   */
  inline void createAndStoreBucket(std::vector<Var>& varConnected, T& c) {
    CachedBucket<T>* formulaBucket =
        this->m_bucketManager->collectBuckect(varConnected);
    unsigned int hashValue = computeHash(*formulaBucket);
    pushInHashTable(*formulaBucket, hashValue, c);
  }

  /**
   * @brief Init the hashTable.
   */
  void initHashTable(unsigned maxVar) override {
    this->setInfoFormula(maxVar);

    // Init the flat structures
    hashTable.assign(SIZE_HASH, END_OF_LIST);
    nodePool.clear();
    freeListHead = END_OF_LIST;

    // Pre-allocate chunk to avoid mid-solve resizing (saves memory reallocation
    // overhead)
    nodePool.reserve(SIZE_HASH);
  }

  /**
   * @brief Clean up the cache.
   */
  unsigned removeEntry(std::function<bool(CachedBucket<T>& c)> test) {
    unsigned nbRemoveEntry = 0;

    for (uint64_t i = 0; i < SIZE_HASH; i++) {
      uint32_t* currPtr = &hashTable[i];

      while (*currPtr != END_OF_LIST) {
        uint32_t currIdx = *currPtr;
        CollisionNode& node = nodePool[currIdx];

        if (test(node.bucket)) {
          assert((int)node.bucket.dataBucket.szData() > 0);

          // Free the underlying DPLL memory
          this->releaseMemory(node.bucket.dataBucket.data,
                              node.bucket.dataBucket.szData());
          node.bucket.dataBucket.reset();
          nbRemoveEntry++;

          // Unlink from the hash table list
          *currPtr = node.next;

          // Push this slot to the Free List so pushInHashTable can reuse it
          node.next = freeListHead;
          freeListHead = currIdx;
        } else {
          // If not removed, advance our pointer tracker to the next node
          currPtr = &node.next;
        }
      }
    }
    return nbRemoveEntry;
  }
};
}  // namespace d4
