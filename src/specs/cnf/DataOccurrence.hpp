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

#include <cassert>

namespace d4 {
struct IteratorIdxClause {
  int *start, *end;
  unsigned size() { return end - start; }
};

struct DataOccurrence {
  int *bin;
  unsigned nbBin;
  int *notBin;
  unsigned nbNotBin;

  inline void removeMarkedBin(const std::vector<bool> &markedClauses) {
    if (!nbBin) return;
    int *end = &bin[nbBin - 1];
    while (end >= bin) {
      if (!markedClauses[*end])
        end--;
      else {
        std::swap(*end, *bin);
        bin++;
        nbBin--;
      }
    }
  }

  inline void removeBin(int idx) {
    for (unsigned i = 0; i < nbBin; i++) {
      if (bin[i] == idx) {
        std::swap(bin[i], *bin);
        bin++;
        nbBin--;
        return;
      }
    }
    assert(0);  // we have to remove one element.
  }

  inline void removeNotBin(int idx) {
    for (unsigned i = 0; i < nbNotBin; i++) {
      if (notBin[i] == idx) {
        std::swap(notBin[i], notBin[nbNotBin - 1]);
        --nbNotBin;
        return;
      }
    }
    assert(0);  // we have to remove one element.
  }

  inline void addBin(int idx) {
    --bin;
    *bin = idx;
    nbBin++;
  }
  inline void addNotBin(int idx) { notBin[nbNotBin++] = idx; }

  inline IteratorIdxClause getBinClauses() { return {bin, &bin[nbBin]}; }
  inline IteratorIdxClause getNotBinClauses() {
    return {notBin, &notBin[nbNotBin]};
  }
  inline IteratorIdxClause getClauses() { return {bin, &notBin[nbNotBin]}; }
};
}  // namespace d4