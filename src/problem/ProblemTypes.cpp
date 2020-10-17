#include "ProblemTypes.hpp"

namespace d4
{

/**
   Constructor.

   \param[in] v, the variable
   \param[in] sign, the polarity
 */
Lit::Lit(Var v, bool sign)
{
  x = (v << 1) + sign;
} // constructor


/**
   Redefinition of the toString method.
 */
std::ostream& operator<< (std::ostream &os, Lit l)
{
  os << (l.sign() ? -l.var() : l.var());
  return os;
} // redefinition of <<

} // d4
