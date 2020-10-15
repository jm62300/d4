#ifndef d4_circuit_Circuit_hpp
#define d4_circuit_Circuit_hpp

#include <iostream>


namespace d4
{
class Circuit
{
  virtual std::ostream &print(std::ostream &os) = 0;
  
 private:
  int counterFather = 0;

 public:
  virtual ~Circuit(){ }

  friend std::ostream& operator<< (std::ostream &os, Circuit &c)
  {
    return c.print(os);
  }

  inline int getCounter(){return counterFather;}
  inline int incCounter(){return ++counterFather;}
  inline int decCounter(){return --counterFather;}
};
}

#endif
