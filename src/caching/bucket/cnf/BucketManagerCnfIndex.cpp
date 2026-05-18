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

#include "BucketManagerCnfIndex.hpp"

namespace d4 {
/**
 * @brief BucketManagerCnfIndex::BucketManagerCnfIndex implementation.
 */
BucketManagerCnfIndex::BucketManagerCnfIndex(CnfManager& occM,
                                             ModeStore mdStore,
                                             unsigned long sizeFirstPage,
                                             unsigned long sizeAdditionalPage,
                                             BucketAllocator* bucketAllocator)
    : BucketManagerCnf::BucketManagerCnf(occM, mdStore, sizeFirstPage,
                                         sizeAdditionalPage, bucketAllocator) {
}  // BucketManagerCnfIndex

/**
 * @brief BucketManagerCnfIndex::~BucketManagerCnfIndex implementation.
 */
BucketManagerCnfIndex::~BucketManagerCnfIndex() {}  // destructor

/**
 * @brief BucketManagerCnfIndex::storeData implementation.
 */
template <typename U, typename W>
void* BucketManagerCnfIndex::storeData(void* data, std::span<const W> value) {
  U* p = static_cast<U*>(data);
  for (auto& v : value) {
    *p = static_cast<U>(v);
    p++;
  }

  return p;
}  // storeVariables

/**
 * @brief BucketManagerCnfIndex::storeFormula implementation.
 */
void BucketManagerCnfIndex::storeFormula(std::span<const Var> component,
                                         DataBucket& b) {
  this->collectIdActiveClauses(component, m_idxClauses);

  // nb bytes we need to store the information.
  unsigned int nbOVar = this->nbOctetToEncodeInt(component.back() + 1);
  unsigned int nbOData = m_idxClauses.size()
                             ? this->nbOctetToEncodeInt(m_idxClauses.back() + 1)
                             : 1;

  // ask for memory
  unsigned szData = nbOVar * component.size() + nbOData * m_idxClauses.size();
  char* data = this->m_bucketAllocator->getArray(szData);
  void* p = data;

  // store the variables
  switch (nbOVar) {
    case 1:
      p = storeData<uint8_t, Var>(p, component);
      break;
    case 2:
      p = storeData<uint16_t, Var>(p, component);
      break;
    default:
      p = storeData<uint32_t, Var>(p, component);
      break;
  }
  assert(static_cast<char*>(p) == &data[nbOVar * component.size()]);
  if (!m_idxClauses.size()) goto fillTheBucket;

  // store the clauses
  switch (nbOData) {
    case 1:
      p = storeData<uint8_t, unsigned>(p, m_idxClauses);
      break;
    case 2:
      p = storeData<uint16_t, unsigned>(p, m_idxClauses);
      break;
    default:
      p = storeData<uint32_t, unsigned>(p, m_idxClauses);
      break;
  }

fillTheBucket:
  DataInfo di(szData, component.size(), nbOVar, nbOData);
  assert(di.szData() == szData);
  b.set(data, di);
}  // storeFormula
}  // namespace d4