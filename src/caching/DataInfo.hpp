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
#pragma once
#include <bits/stdint-uintn.h>
#include <cassert>
#include <iostream>

#include <cstdint>
#include <math.h>
#include <stdio.h>

#define MASK_SIZE (~((((uint64_t)1 << 21) - 1) << 21))

namespace d4 {
class DataInfo {
protected:
  // we reserve 96 bytes to store information in the cached bucket
  //   We always at least have the following distribution:
  // info1 => |free(17)| nbBitVar(5)|nb var(21)|szData(21)|
  // For info2 => |szData (26 bytes)|nb octets data (2 bytes)|nb
  // octets var (2 bytes)| free (2 bytes)|
public:
  uint64_t info1;
  uint32_t info2;

  DataInfo();
  DataInfo(unsigned szData, unsigned nbVar, unsigned nbOctetsData,
           unsigned nbBitVar, unsigned count);

  inline unsigned *getInfo() { return (unsigned *)this; }
  inline unsigned getSizeInfo() { return 3; }

  bool operator==(const DataInfo &d) const {
    return info1 == d.info1 && info2 == d.info2;
  } // operator ==

  virtual ~DataInfo() {}

  inline unsigned szData() {
    return ((uint64_t)info1 >> 21) & (((uint64_t)1 << 21) - 1);
  }
  inline unsigned nbVar() {
    return (uint64_t)info1 & (((uint64_t)1 << 21) - 1);
  }

  inline void szData(unsigned sz) {
    info1 &= ((uint64_t)sz << 21) | MASK_SIZE;
    assert(szData() == sz);
  }

  inline unsigned nbOctetsVar() { return 1 + ((info2 >> 2) & ((1 << 2) - 1)); }
  inline unsigned nbOctetsData() { return 1 + ((info2 >> 4) & ((1 << 2) - 1)); }

  inline void reset() { info1 = 0; }

  virtual void print(char *data, std::ostream &out) { out << data; }

  template <typename U> void printData(void *data, int sz, std::ostream &out) {
    U *p = static_cast<U *>(data);
    for (int i = 0; i < sz; i++) {
      out << (unsigned)*p << " ";
      p++;
    }
    out << "\n";
  } // printdata
};
} // namespace d4
