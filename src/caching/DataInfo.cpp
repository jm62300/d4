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
#include "DataInfo.hpp"

#include <bits/stdint-uintn.h>

#include <bitset>

namespace d4 {
/**
   Constructor.
 */
DataInfo::DataInfo() { info1 = 0; }  // constructor

/**
 * @brief Construct a new Data Info:: Data Info object
 *
 * @param szData
 * @param nbVar
 * @param nbOctetsData
 * @param nbOctetsVar
 */
DataInfo::DataInfo(unsigned szData, unsigned nbVar, unsigned nbBitVar,
                   unsigned nbBitFormula) {
  info = {nbBitFormula, nbBitVar, nbVar, szData};

  assert(nbBitFormula < (1 << 5));
  assert(nbBitVar < (1 << 5));
  assert(nbVar == this->nbVar());
  assert(szData == this->szData());
}  // constructor

}  // namespace d4