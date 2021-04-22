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

#include "ScoringMethodJwts.hpp"

namespace d4 {

/**
   Constructor.

   @param[in] om, the manager that give information about the CNF formula.
 */
ScoringMethodJwts::ScoringMethodJwts(SpecManagerCnf &o)
    : om(o) {} // constructor

/**
   This scoring function favorises the varaibles which appear in
   most clauses.

   R. G. Jeroslow and J. Wang. Solving propositional satisfiability
   problems. Annals of Mathematics and Artificial Intelligence,
   1:167–187, 1990.

   @param[in] v, the variable we want the score.
 */
double ScoringMethodJwts::computeScore(Var v) {
  Lit lp = Lit::makeLit(v, false);
  double res =
      om.getVecIdxClauseBin(lp).size() + om.getVecIdxClauseBin(~lp).size();
  res /= 4;

  for (auto &idx : om.getVecIdxClauseNotBin(lp)) {
    assert(!om.isSatisfiedClause(idx));
    if (om.getInitSize(idx) > 5)
      continue;
    res += ((double)1.0) / (1 << om.getCurrentSize(idx));
  }

  for (auto &idx : om.getVecIdxClauseNotBin(~lp)) {
    assert(!om.isSatisfiedClause(idx));
    if (om.getInitSize(idx) > 5)
      continue;
    res += ((double)1.0) / (1 << om.getCurrentSize(idx));
  }

  return res;
} // computeScore

} // namespace d4
