#ifndef d4_circuit_ClauseGate_hpp
#define d4_circuit_ClauseGate_hpp

#include <iostream>
#include <vector>

#include "Circuit.hpp"

namespace d4
{
class ClauseGate : public Circuit
{
  std::ostream &print(std::ostream &os);
  
 private:
  std::vector<int> literals_;
  
 public:
  ClauseGate(std::vector<int> &literals);
  ~ClauseGate();
};
}
#endif
