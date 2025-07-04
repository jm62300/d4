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

#include "BucketManagerCnfCombi.hpp"

namespace d4 {
/**
 * @brief BucketManagerCnfCombi::BucketManagerCnfCombi implementation.
 */
BucketManagerCnfCombi::BucketManagerCnfCombi(
    CnfManager &occM, ModeStore mdStore, unsigned long sizeFirstPage,
    unsigned long sizeAdditionalPage, unsigned limitNbVarSym,
    unsigned limitNbVarIndex, BucketAllocator *bucketAllocator)
    : BucketManagerCnf::BucketManagerCnf(occM, mdStore, sizeFirstPage,
                                         sizeAdditionalPage, bucketAllocator),
      m_inConstruction(occM) {
  clBucketManager =
      new BucketManagerCnfCl(occM, mdStore, sizeFirstPage, sizeAdditionalPage,
                             this->m_bucketAllocator);

  clBucketManagerBis =
      new BucketManagerCnfCl(occM, mdStore, sizeFirstPage, sizeAdditionalPage,
                             this->m_bucketAllocator);

  symBucketManager =
      new BucketManagerCnfSym(occM, mdStore, sizeFirstPage, sizeAdditionalPage,
                              this->m_bucketAllocator);

  indexBucketManager =
      new BucketManagerCnfIndex(occM, mdStore, sizeFirstPage,
                                sizeAdditionalPage, this->m_bucketAllocator);

  m_limitNbVarIndex = limitNbVarIndex;
  m_limitNbVarSym = limitNbVarSym;
  this->m_bucketAllocator->deactiveCleanUp();
}  // BucketManagerCnfCombi

/**
 * @brief BucketManagerCnfCombi::~BucketManagerCnfCombi implementation.
 */
BucketManagerCnfCombi::~BucketManagerCnfCombi() {
  delete symBucketManager;
  delete indexBucketManager;
  delete clBucketManagerBis;
  delete clBucketManager;
  this->m_bucketAllocator->activeCleanUp();
}  // destructor

/**
 * @brief BucketManagerCnfCombi::storeFormula implementation.
 */
void BucketManagerCnfCombi::storeFormula(std::vector<Var> &component,
                                         DataBucket &b) {
  if (component.size() < m_limitNbVarSym)
    return symBucketManager->storeFormula(component, b);
  if (component.size() > m_limitNbVarIndex)
    return indexBucketManager->storeFormula(component, b);
  return clBucketManager->storeFormula(component, b);
}  // storeFormula
}  // namespace d4