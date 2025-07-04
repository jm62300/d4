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

#include <vector>

#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"

namespace d4 {

class OptionPreprocManager;

enum PreprocMethod {
  BASIC,
  BACKBONE,
  EQUIV,
  SHARP_EQUIV,
  VIVI,
  OCC_ELIM,
  COMPILE_EQUIV,
  COMB
};

class PreprocMethodManager {
 public:
  static std::string getPreprocMethod(const PreprocMethod &m) {
    if (m == BASIC) return "basic";
    if (m == EQUIV) return "equiv";
    if (m == BACKBONE) return "backbone";
    if (m == SHARP_EQUIV) return "#equiv";
    if (m == VIVI) return "vivification";
    if (m == OCC_ELIM) return "occElimination";
    if (m == COMB) return "combinaison";
    if (m == COMPILE_EQUIV) return "compile-equiv";

    throw(FactoryException("Preproc Method unknown", __FILE__, __LINE__));
  }  // getInputType

  static PreprocMethod getPreprocMethod(const std::string &m) {
    if (m == "basic") return BASIC;
    if (m == "equiv") return EQUIV;
    if (m == "backbone") return BACKBONE;
    if (m == "sharp-equiv") return SHARP_EQUIV;
    if (m == "vivification") return VIVI;
    if (m == "occElimination") return OCC_ELIM;
    if (m == "combinaison") return COMB;
    if (m == "compile-equiv") return COMPILE_EQUIV;

    throw(FactoryException("Preproc Method unknown", __FILE__, __LINE__));
  }  // getInputType
};

class PreprocManager {
 public:
  static void *s_isRunning;
  virtual ~PreprocManager() {}

  static PreprocManager *makePreprocManager(const OptionPreprocManager &options,
                                            std::ostream &out);

  virtual ProblemManager *run(ProblemManager *pin,
                              const OptionPreprocManager &optionPreproc) = 0;
};
}  // namespace d4
