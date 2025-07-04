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

#include "BucketManagerCnf.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {
class BucketManagerCnf;
class BucketManagerCnfIndex : public BucketManagerCnf {
 private:
  std::vector<unsigned> m_idxClauses;

 protected:
  /**
       Store the variables respecting the information of size concerning the
     type T to encode each elements and returns the pointer just after the end
     of the data.

       @param[]
    */
  template <typename U, typename W>
  void *storeData(void *data, std::vector<W> &value);

 public:
  /**
     Function called in order to initialized variables before using

     @param[in] occM, the CNF occurrence manager
     @param[in] mdStore, the storing mode for the clause
     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional
     pages.
  */
  BucketManagerCnfIndex(
      CnfManager &occM, ModeStore mdStore, unsigned long sizeFirstPage,
      unsigned long sizeAdditionalPage,
      BucketAllocator *bucketAllocator = new BucketAllocator());

  /**
     Destructor.
   */
  ~BucketManagerCnfIndex();

  /**
     Transfer the formula store in distib in a table given in parameter.

     @param[in] component, the input variables
     @param[out] tmpFormula, the place where is stored the formula
     @param[out] szTmpFormula, to collect the size of the stored formula
  */
  void storeFormula(std::vector<Var> &component, DataBucket &b) override;
};
}  // namespace d4
