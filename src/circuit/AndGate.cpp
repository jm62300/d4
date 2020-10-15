#include "AndGate.hpp"

namespace d4
{

/**
   Constructor.
   
   @param[in] children, the nodes connected to the and node.
 */
AndGate::AndGate(std::vector<Circuit *> children) : children_(children)
{
  for(auto s : children_) s->incCounter();
} // constructor


AndGate::~AndGate()
{
  for(auto n : children_) if(!n->decCounter()) delete n;
} // destructor


/**
   Redefinition of the toString operator.
   
   @param[in] os, the stream where will be write the information.
   
   \return the modified stream.
 */
std::ostream & AndGate::print(std::ostream &os)
{  
  os << "AND(";
  for (unsigned i = 0 ; i<children_.size() ; i++)
    os << ((!i) ? "" : ", ") << *children_[i];
  os << ")";
  return os;
} // redefine <<

}
