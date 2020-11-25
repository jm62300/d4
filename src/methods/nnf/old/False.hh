#ifndef DAG_False_h
#define DAG_False_h

#include "DAG.hh"

template<class T> class DAG;
template<class T> class falseNode : public DAG<T>
{
public:
  using DAG<T>::globalStamp;
  using DAG<T>::idxOutputStruct;
  using DAG<T>::stamp;


  inline void printNNF(std::ostream& out, bool certif)
  {
    if(stamp >= globalStamp) return;
    stamp = globalStamp + idxOutputStruct + 1;
    int idxCurrent = ++idxOutputStruct;

    out << "f " << idxCurrent << " 0" << endl;
  }

  inline bool isSAT(vec<Lit> &unitsLitBranches) {return false;}
  inline T computeNbModels() { return 0; }
};

#endif
