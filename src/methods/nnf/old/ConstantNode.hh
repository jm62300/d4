#ifndef DAG_ConstantNode_h
#define DAG_ConstantNode_h

#include "DAG.hh"

template<class T> class DAG;
template<class T> class ConstantNode : public DAG<T>
{
public:
  T nbModels;

  ConstantNode(T _nbModels) : nbModels(_nbModels) {}
  
  inline void toNNF(std::ostream& out)
  {
    assert(false); // This should never get called since is a singleton
    out << "A 0" << std::endl;
  }

  inline void printNNF(){out << T;}// printNNF

  inline bool isSAT(){ return (nbModels > 0);}
  inline T computeNbModels() { return nbModels; }
};

#endif
