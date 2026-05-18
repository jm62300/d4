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

#include <sys/types.h>

#include <cstddef>
#include <iostream>
#include <typeinfo>

namespace d4 {
class HashString {
 private:
  static inline uint64_t MurmurHash64A(const void* key, int len,
                                       uint64_t seed) {
    const uint64_t m = 0xc6a4a7935bd1e995ULL;
    const int r = 47;

    uint64_t h = seed ^ (len * m);

    const uint64_t* data = (const uint64_t*)key;
    const uint64_t* end = data + (len / 8);

    while (data != end) {
      uint64_t k = *data++;

      k *= m;
      k ^= k >> r;
      k *= m;

      h ^= k;
      h *= m;
    }

    const uint8_t* data2 = (const uint8_t*)data;

    switch (len & 7) {
      case 7:
        h ^= uint64_t(data2[6]) << 48;
      case 6:
        h ^= uint64_t(data2[5]) << 40;
      case 5:
        h ^= uint64_t(data2[4]) << 32;
      case 4:
        h ^= uint64_t(data2[3]) << 24;
      case 3:
        h ^= uint64_t(data2[2]) << 16;
      case 2:
        h ^= uint64_t(data2[1]) << 8;
      case 1:
        h ^= uint64_t(data2[0]);
        h *= m;
    };

    h ^= h >> r;
    h *= m;
    h ^= h >> r;

    return h;
  }

 public:
  inline uint64_t hash(char* key, unsigned len, uint64_t info) {
    uint64_t dataHash = MurmurHash64A(key, len, 29111983ULL);
    uint64_t infoHash = MurmurHash64A(&info, sizeof(uint64_t), 30011989ULL);

    uint64_t h = dataHash ^ infoHash;
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33;
    h *= 0xc4ceb9fe1a85ec53ULL;
    h ^= h >> 33;

    return h;
  }  // hash
};
}  // namespace d4
