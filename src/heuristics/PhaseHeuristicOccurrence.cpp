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

#include "PhaseHeuristicOccurrence.hpp"

namespace d4
{

/**
   Constructor.

   @param[in] s, the manager that give information about the formula.
 */
PhaseHeuristicOccurrence::PhaseHeuristicOccurrence(SpecManager &s) : sm(s)
{
  
} // constructor


/**
   Assign the next decision variable regarding the number of occurrence of the
   variable in the formula.
 */
bool PhaseHeuristicOccurrence::selectPhase(Var v)
{
  return sm.getNbOccurrence(Lit(v, false)) < sm.getNbOccurrence(Lit(v, true));
} // selectPhase

}
