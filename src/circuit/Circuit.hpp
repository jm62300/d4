#ifndef d4_circuit_Circuit_hpp
#define d4_circuit_Circuit_hpp

#include <iostream>


namespace d4
{
class Circuit
{
  virtual std::ostream &print(std::ostream &os) = 0;
  
 private:

 public:
  virtual ~Circuit(){ }

  friend std::ostream& operator<< (std::ostream &os, Circuit &c)
  {
    return c.print(os);
  }
};
}

#endif
