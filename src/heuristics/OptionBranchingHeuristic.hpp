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

#include <string>

#include "src/exceptions/FactoryException.hpp"

namespace d4 {
enum ScoringMethodType {
  SCORE_MOM,
  SCORE_DLCS,
  SCORE_VSIDS,
  SCORE_VSADS,
  SCORE_JWTS
};

class ScoringMethodTypeManager {
 public:
  static std::string getScoringMethodType(const ScoringMethodType& m) {
    if (m == SCORE_MOM) return "mom";
    if (m == SCORE_DLCS) return "dlcs";
    if (m == SCORE_VSIDS) return "vsids";
    if (m == SCORE_VSADS) return "vsads";
    if (m == SCORE_JWTS) return "jwts";

    throw(FactoryException("Scoring method type unknown", __FILE__, __LINE__));
  }  // getScoringMethodType

  static ScoringMethodType getScoringMethodType(const std::string& m) {
    if (m == "mom") return SCORE_MOM;
    if (m == "dlcs") return SCORE_DLCS;
    if (m == "vsids") return SCORE_VSIDS;
    if (m == "vsads") return SCORE_VSADS;
    if (m == "jwts") return SCORE_JWTS;

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getScoringMethodType
};

class OptionScoringMethod {
 public:
  ScoringMethodType scoringMethodType;

  friend std::ostream& operator<<(std::ostream& out,
                                  const OptionScoringMethod& dt) {
    out << " Option Scoring Method:"
        << " meth("
        << ScoringMethodTypeManager::getScoringMethodType(dt.scoringMethodType)
        << ")";
    return out;
  }  // <<
};
}  // namespace d4