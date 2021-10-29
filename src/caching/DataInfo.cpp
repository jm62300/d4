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
#include <bitset>

#include "DataInfo.hpp"

namespace d4 {
/**
   Constructor.
 */
DataInfo::DataInfo() { info1 = info2 = 0; } // constructor

/**
 * @brief Construct a new Data Info:: Data Info object
 *
 * @param szData
 * @param nbVar
 * @param nbOctetsData
 * @param nbOctetsVar
 * @param count
 */
DataInfo::DataInfo(unsigned szData, unsigned nbVar, unsigned nbOctetsData,
                   unsigned nbOctetsVar, unsigned count) {
  info1 = info2 = 0;
  assert((nbOctetsData - 1) < 4);
  assert((nbOctetsVar - 1) < 4);

  info1 = (uint64_t)nbVar;
  info2 = ((uint32_t)(nbOctetsData - 1) << 4) |
          ((uint32_t)(nbOctetsVar - 1) << 2) | ((uint32_t)szData << 6);
  assert(szData == this->szData());
} // constructor

} // namespace d4
