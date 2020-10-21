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

#ifndef d4_src_problem_cnf_DynamicOccurrenceManager_hpp
#define d4_src_problem_cnf_DynamicOccurrenceManager_hpp

#include <vector>
#include <cassert>

#include <problem/ProblemTypes.hpp>
#include <problem/ProblemManager.hpp>

#include "SpecManagerCnf.hpp"


namespace d4
{
class SpecManagerCnfDyn : public SpecManagerCnf
{
 private:
  void initClauses(std::vector<std::vector<Lit> > &clauses);

 public:
  SpecManagerCnfDyn(int nbClause, int nbVar, int maxClauseSize);
  SpecManagerCnfDyn(ProblemManager &p);

  void preUpdate(std::vector<Lit> &lits);
  void postUpdate(std::vector<Lit> &lits);
  void removeIdxFromOccList(std::vector<int> &o, int idx);

  // we cannot use this function here
  inline void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units){assert(0);}
  void initFormula(ProblemManager &p);
};

}

#endif
