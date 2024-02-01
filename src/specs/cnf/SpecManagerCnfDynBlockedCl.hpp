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
#include <src/problem/ProblemManager.hpp>
#include <src/problem/ProblemTypes.hpp>
#include <vector>

#include "SpecManagerCnf.hpp"

namespace d4 {

class SpecManagerCnfDynBlockedCl : public SpecManagerCnf {
  struct SavedStateOcc {
    Lit l;
    unsigned nbBin;
    unsigned nbNotBin;
  };

  struct SavedStateClause {
    int idx;
    unsigned isSat : 1;
    unsigned nbUnsat : 31;
  };

 private:
  std::vector<int> m_reviewWatcher;
  std::vector<char> m_markedLit;
  std::vector<bool> m_markedClauseIdx;

  std::vector<SavedStateOcc> m_savedStateOccs;
  std::vector<SavedStateClause> m_savedStateClauses;
  std::vector<unsigned> m_stackPosOcc, m_stackPosClause;

  std::vector<bool> m_isDecisionVariable;
  std::vector<bool> m_markedPureLiteral;
  std::vector<Lit> m_pureDetected;

  void initClauses(std::vector<std::vector<Lit>> &clauses);

 public:
  SpecManagerCnfDynBlockedCl(ProblemManager &p);

  void preUpdate(std::vector<Lit> &lits);
  void postUpdate(std::vector<Lit> &lits);

  // we cannot use this function here
  inline void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units) {
    assert(0);
  }
};
}  // namespace d4
