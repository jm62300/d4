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

#include "DataInfo.hpp"

namespace d4
{
/**
   Constructor.
 */
DataInfo::DataInfo()
{
  info1 = info2 = 0;
  stats = {0,0};
} // constructor

/**
   Constructor.

   We always at least have the following distribution:
   For info1 => |free (43 bytes)|nb var (21 bytes)|
   For info2 => |szData (26 bytes)|nb octets data (2 bytes)|nb octets var (2 bytes)| free (2 bytes)|

   @param[in] i1, the information place 1.
   @param[in] i2, the information place 2.
   @param[in] count, the counter value for initialization.
*/
DataInfo::DataInfo(uint64_t i1, uint32_t i2, unsigned count)
{
  info1 = i1;
  info2 = i2;
  stats = {count, 0};
} // constructor

}
