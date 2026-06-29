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

#include <cassert>
#include <deque>
#include <iostream>
#include <vector>

#include "src/utils/MemoryStat.hpp"

namespace d4 {

class BucketAllocator {
 private:
  std::vector<char*> m_allocateData;
  char* m_data = NULL;
  unsigned long m_sizeFirstPage;
  unsigned long m_sizeAdditionalPage;
  unsigned long m_sizeData;
  unsigned long m_posInData;

  // freespace[i][j] points to a free memory space of size i
  std::vector<std::vector<char*>> m_freeSpace;
  unsigned long int m_allMemory;
  unsigned long int m_freeMemory;
  unsigned long int m_usedMemory;
  bool isInit = false;
  bool cleanup = true;
  bool m_consumedMemory = false;

  unsigned long int m_memoryLimit = 10000 * (1ul << 20);
  unsigned long int m_memoryLimitInc = 5000 * (1ul << 20);

  // The memory limit is checked against the process Resident Set Size (the
  // physical memory the OS actually has resident), not against m_allMemory
  // (which only accounts for the pages handed out by this allocator). Reading
  // the RSS means parsing /proc/self/status, which is far too costly for the
  // hot path, so the value is cached and only refreshed once every
  // MEM_CHECK_PERIOD calls to isMemoryLimitReached().
  static constexpr unsigned MEM_CHECK_PERIOD = 256;
  unsigned m_memCheckCounter = 0;
  unsigned long int m_residentMemoryBytes = 0;

 public:
  ~BucketAllocator() {
    for (auto data : m_allocateData) delete[] data;
    m_allocateData.clear();
  }

  inline bool getComsumedMemory() { return m_consumedMemory; }
  inline void reinitComsumedMemory() {
    m_memoryLimit += m_memoryLimitInc;
    m_consumedMemory = false;
  }

  inline void activeCleanUp() { cleanup = true; }
  inline void deactiveCleanUp() { cleanup = false; }
  inline bool getCleanup() { return cleanup; }
  inline bool getIsInit() { return isInit; }
  inline void setIsInit(bool v) { isInit = v; }

  inline unsigned long int usedMemory() { return m_usedMemory; }

  /**
   * @brief True when the process resident memory (RSS) exceeds the limit.
   *
   * The RSS is the real physical memory the system holds for this process, so
   * it captures everything (cache, SAT solver, GMP integers, heap
   * fragmentation, ...), not just the cache pages tracked by m_allMemory. The
   * actual /proc read is throttled to once every MEM_CHECK_PERIOD calls; the
   * remaining calls compare against the cached value, keeping the common path
   * a plain integer comparison.
   */
  inline bool isMemoryLimitReached() {
    if (!m_memCheckCounter--) {
      m_memCheckCounter = MEM_CHECK_PERIOD;
      m_residentMemoryBytes =
          (unsigned long int)(MemoryStat::memResident() * (double)(1ul << 20));
    }
    return m_residentMemoryBytes > m_memoryLimit;
  }  // isMemoryLimitReached

  inline double remainingMemory() {
    return ((double)m_freeMemory + (m_sizeData - m_posInData)) /
           (double)m_allMemory;
  }  // remainingMemory

  void init(unsigned long sizeFirstPage, unsigned long sizeAdditionalPage);

  char* getArray(unsigned size);

  void releaseMemory(char* m, unsigned size);
};

}  // namespace d4
