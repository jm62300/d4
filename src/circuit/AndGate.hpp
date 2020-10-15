#ifndef d4_circuit_AndGate_hpp
#define d4_circuit_AndGate_hpp

#include <vector>

#include "Circuit.hpp"

namespace d4
{
class AndGate : public Circuit
{
  std::ostream &print(std::ostream &os);

 private:
  std::vector<Circuit *> children_;

 public:
  AndGate(std::vector<Circuit *> children);
  ~AndGate();
};
}

#endif
