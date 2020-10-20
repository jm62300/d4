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

#ifndef d4_src_problem_ProblemTypes_hpp
#define d4_src_problem_ProblemTypes_hpp

#include <iostream>
#include <vector>

namespace d4
{

typedef int Var;
typedef uint8_t lbool;

#define var_Undef (-1)
#define l_True  (lbool((uint8_t)0)) // gcc does not do constant propagation if these are real constants.
#define l_False (lbool((uint8_t)1))
#define l_Undef (lbool((uint8_t)2))

class Lit
{
 public:
  int x;
  
  Lit(Var v, bool sign = false);

  inline bool sign(){return x&1;}
  inline Var var(){return x>>1;}
  inline Lit neg(){return Lit(x>>1, (x+1) & 1);}
  inline int intern(){return x;}
  inline int human(){return (x&1) ? -var() : var();}
  
  bool operator == (Lit p) const { return x == p.x; }
  bool operator != (Lit p) const { return x != p.x; }
  bool operator <  (Lit p) const { return x < p.x;  } // '<' makes p, ~p adjacent in the ordering.

  friend std::ostream& operator<< (std::ostream &os, Lit l);
};

const Lit lit_Undef = { -2 };  // }- Useful special constants.
const Lit lit_Error = { -1 };  // }

inline void showListLit(std::ostream &out, std::vector<Lit> &v)
{
  for(auto &l : v) out << l << " ";
} // showListLit


inline Lit operator ~(Lit p) {return Lit(p.x, !(p.sign()));}


}

#endif
