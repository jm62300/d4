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
#include <bits/stdint-uintn.h>
#include <math.h>
#include <stdio.h>

#include <bitset>
#include <cassert>
#include <cstdint>
#include <iostream>

namespace d4 {
class DataInfo {
 protected:
 public:
  union {
    struct {
      unsigned nbBitFormula : 5;
      unsigned nbBitVar : 5;
      unsigned nbVar : 22;
      unsigned szData : 32;
    } info;
    u_int64_t info1;
  };

  DataInfo();
  DataInfo(unsigned szData, unsigned nbVar, unsigned nbBitVar,
           unsigned nbBitFormula);

  inline unsigned *getInfo() { return (unsigned *)this; }
  inline unsigned getSizeInfo() { return 3; }

  bool operator==(const DataInfo &d) const {
    return info1 == d.info1;
  }  // operator ==

  virtual ~DataInfo() {}

  inline unsigned szData() { return info.szData; }
  inline unsigned nbVar() { return info.nbVar; }

  inline void szData(unsigned sz) {
    info.szData = sz;
    assert(szData() == sz);
  }

  inline void reset() { info1 = 0; }

  template <typename U>
  void printData(void *data, int sz, std::ostream &out) {
    char *p = (char *)data;
    for (int i = 0; i < sz; i++) {
      out << std::bitset<8>(p[i]) << " ";
    }
    out << "\n";
  }  // printdata
};
}  // namespace d4
