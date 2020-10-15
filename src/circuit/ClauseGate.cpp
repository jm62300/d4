#include "ClauseGate.hpp"


namespace d4
{
/**
   Constructor.

   @param[in] literals, the set of literals belonging to the clause.
 */
ClauseGate::ClauseGate(std::vector<int> &literals) : literals_(literals)
{
  
} // constructor


/**
   Destructor.
 */
ClauseGate::~ClauseGate()
{
  
} // destructor

/**
   Redefinition of the toString operator.
   
   @param[in] os, the stream where will be write the information.
   
   \return the modified stream.
 */
std::ostream & ClauseGate::print(std::ostream &os)
{  
  os << "CL(";
  for (unsigned i = 0 ; i<literals_.size() ; i++)
    os << ((!i) ? "" : ", ") << literals_[i];
  os << ")";
  return os;
} // redefine <<

}
