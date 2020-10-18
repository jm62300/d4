#ifndef d4_src_problem_ProblemTypes_hpp
#define d4_src_problem_ProblemTypes_hpp

#include <iostream>

namespace d4
{

typedef int Var;
#define var_Undef (-1)

class Lit
{
 public:
  int x;
  
  Lit(Var v, bool sign = false);

  inline bool sign(){return x&1;}
  inline Var var(){return x>>1;}
  inline Lit neg(){return Lit(x>>1, (x+1) & 1);}
  
  bool operator == (Lit p) const { return x == p.x; }
  bool operator != (Lit p) const { return x != p.x; }
  bool operator <  (Lit p) const { return x < p.x;  } // '<' makes p, ~p adjacent in the ordering.

  friend std::ostream& operator<< (std::ostream &os, Lit l);
};


}

#endif
