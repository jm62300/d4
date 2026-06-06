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

#include "FormulaStoreCnfCombi.hpp"

namespace d4 {

FormulaStoreCnfCombi::FormulaStoreCnfCombi(CnfManager& occM, ModeStore mdStore,
                                           unsigned limitNbVarSym,
                                           unsigned limitNbVarIndex)
    : FormulaStoreCnf(occM, mdStore) {
  clBucketManager = new FormulaStoreCnfCl(occM, mdStore);
  clBucketManagerBis = new FormulaStoreCnfCl(occM, mdStore);
  symBucketManager = new FormulaStoreCnfSym(occM, mdStore);
  indexBucketManager = new FormulaStoreCnfIndex(occM, mdStore);
  m_limitNbVarSym = limitNbVarSym;
  m_limitNbVarIndex = limitNbVarIndex;
}

FormulaStoreCnfCombi::~FormulaStoreCnfCombi() {
  delete symBucketManager;
  delete indexBucketManager;
  delete clBucketManagerBis;
  delete clBucketManager;
}

void FormulaStoreCnfCombi::storeFormula(std::span<const Var> component,
                                        DataBucket& b, BucketAllocator& alloc) {
  if (component.size() < m_limitNbVarSym)
    return symBucketManager->storeFormula(component, b, alloc);
  if (component.size() > m_limitNbVarIndex)
    return indexBucketManager->storeFormula(component, b, alloc);
  return clBucketManager->storeFormula(component, b, alloc);
}

}  // namespace d4
