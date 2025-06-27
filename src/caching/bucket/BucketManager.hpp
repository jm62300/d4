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

#include <string.h>

#include <cassert>
#include <deque>
#include <iostream>
#include <vector>

#include "../CachedBucket.hpp"
#include "BucketAllocator.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/formulaManager/FormulaManager.hpp"
#include "src/formulaManager/circuit/CircuitWithCnfManager.hpp"
#include "src/options/cache/OptionBucketManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/utils/ErrorCode.hpp"

namespace d4 {
// forward declaration
class BucketAllocator;

class BucketManager {
 protected:
  BucketAllocator *m_bucketAllocator;

 public:
  virtual ~BucketManager() {
    if (m_bucketAllocator->getCleanup()) delete m_bucketAllocator;
  }  // destructor

  /**
   * @brief Given the options, this function returns the appropiate bucket
   * manager that deals with CNF formula.
   *
   * @param[in] scnf is the CNF manager.
   * @param[in] cache is the cacha manager.
   * @param[in] options are the options.
   *
   * @return a CNf bucket manager.
   */
  static BucketManager *getBucketMangerCnf(CnfManager &scnf,
                                           OptionBucketManager options);

  /**
   * @brief Create a bucket manager regarding the given options.
   *
   * @param options are the options.
   * @param s is the spect manager which is linked to the bucket manager.
   * @param out is the stream where is printed out the logs.
   *
   * @return a bucket manager that fits with given options.
   */
  static BucketManager *makeBucketManager(OptionBucketManager options,
                                          FormulaManager &s, std::ostream &out);

  inline int nbOctetToEncodeInt(unsigned int v) {
    // we know that we cannot have more than 1<<32 variables
    if (v < (1 << 8)) return 1;
    if (v < (1 << 16)) return 2;
    return 4;
  }  // nbOctetToEncodeInt

  /**
   * @brief Compute the number of bit needed to encode an unsigned given in
   * parameter.
   *
   * @param v is the value we search for its number of bits.
   * @return the number of bit needed to encode val (~log2(val)).
   */
  inline unsigned nbBitUnsigned(unsigned v) {
    const unsigned int b[] = {0x2, 0xC, 0xF0, 0xFF00, 0xFFFF0000};
    const unsigned int S[] = {1, 2, 4, 8, 16};
    int i;

    unsigned int r = 0;       // result of log2(v) will go here
    for (i = 4; i >= 0; i--)  // unroll for speed...
    {
      if (v & b[i]) {
        v >>= S[i];
        r |= S[i];
      }
    }

    return r + 1;
  }  // nbBitUnsigned

  /**
   * @brief Collect the bucket associtated to the set of variable given in
   * parameter.
   *
   * @param[in] component, the variable belonging to the connected component
   * @param[in] bucket is the place where are stored the information.
   *
   * \return a formula put in a bucket
   */
  inline void collectBucket(std::vector<Var> &component, DataBucket &bucket) {
    storeFormula(component, bucket);
  }  // collectBuckect

  inline unsigned long int usedMemory() {
    return m_bucketAllocator->usedMemory();
  }

  inline bool getComsumedMemory() {
    return m_bucketAllocator->getComsumedMemory();
  }
  inline void reinitComsumedMemory() {
    m_bucketAllocator->reinitComsumedMemory();
  }

  inline void releaseMemory(char *m, unsigned size) {
    m_bucketAllocator->releaseMemory(m, size);
  }  // releaseMemory

  inline double remainingMemory() {
    return m_bucketAllocator->remainingMemory();
  }  // remainingMemory

  virtual void storeFormula(std::vector<Var> &component, DataBucket &b) = 0;
};
}  // namespace d4
