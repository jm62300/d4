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

#include <algorithm>

#include "FormulaStoreCnf.hpp"
#include "FormulaStoreCnfCl.hpp"
#include "FormulaStoreCnfIndex.hpp"
#include "FormulaStoreCnfSym.hpp"
#include "src/caching/bucket/cnf/BucketInConstruction.hpp"
#include "src/caching/bucket/cnf/BucketSortInfo.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class FormulaStoreCnfCombi : public FormulaStoreCnf {
 private:
  FormulaStoreCnfCl* clBucketManager;
  FormulaStoreCnfCl* clBucketManagerBis;
  FormulaStoreCnfIndex* indexBucketManager;
  FormulaStoreCnfSym* symBucketManager;

  unsigned m_limitNbVarSym;
  unsigned m_limitNbVarIndex;

 public:
  FormulaStoreCnfCombi(CnfManager& occM, ModeStore mdStore,
                       unsigned limitNbVarSym, unsigned limitNbVarIndex);
  ~FormulaStoreCnfCombi();

  void storeFormula(std::span<const Var> component, DataBucket& b,
                    BucketAllocator& alloc) override;
};

}  // namespace d4
