#ifndef DAG_True_h
#define DAG_True_h

#include "DAG.hh"

template<class T> class DAG;
template<class T> class trueNode : public DAG<T>
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

    out << "t " << idxCurrent << " 0" << endl;
  }

  inline bool isSAT(vec<Lit> &unitsLitBranches){return true;}
  inline T computeNbModels() { return 1; }
};

#endif
