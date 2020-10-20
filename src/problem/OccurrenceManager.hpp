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

#ifndef d4_src_problem_OccurrenceManager_hpp
#define d4_src_problem_OccurrenceManager_hpp

#include <vector>
#include "ProblemTypes.hpp"
#include "ProblemManager.hpp"

namespace d4
{
class OccurrenceManager
{
public:
  virtual ~OccurrenceManager(){}
  virtual bool litIsAssigned(Lit l) = 0;
  virtual bool litIsAssignedToTrue(Lit l) = 0;
  virtual bool varIsAssigned(Var v) = 0;
  virtual int computeConnectedComponent(std::vector<std::vector<Var> > &varConnected, std::vector<Var> &setOfVar,
                                        std::vector<Var> &freeVar, std::vector<Var> &notFreeVar) = 0;
  virtual void preUpdate(std::vector<Lit> &lits) = 0;
  virtual void postUpdate(std::vector<Lit> &lits) = 0;
  virtual void initialize(std::vector<Var> &setOfVar, std::vector<Lit> &units) = 0;
  virtual void showFormula(std::ostream &out) = 0;
  virtual void initFormula(ProblemManager &p) = 0;
  virtual bool byPass(int mode, int idx) = 0;
};
}
#endif
