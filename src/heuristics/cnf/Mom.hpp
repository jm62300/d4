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

#ifndef d4_src_heuristics__cnf_Mom_hpp
#define d4_src_heuristics__cnf_Mom_hpp

#include "../../problem/cnf/CnfOccurrenceManager.hpp"
#include "../ScoringMethod.hpp"

namespace d4
{
class Mom : public ScoringMethod
{
  private:
  CnfOccurrenceManager *om;
  
public:
  Mom(CnfOccurrenceManager *_om);
};
}

#endif
