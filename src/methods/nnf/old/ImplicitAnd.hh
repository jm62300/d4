#ifndef Minisat_DAG_ImplicitAnd_h
#define Minisat_DAG_ImplicitAnd_h

#include "../DAG/DAG.hh"


template<class T> class DAG;
// This is a class used only to generate the d-DNNF file
// It represents implicit "and" nodes.
template <class T> class ImplicitAnd: public DAG<T>
{
public:
  ImplicitAnd(Branch<T>& b): b{b}
  {
    DAG<T>::nbImplicitAnd++;
    if (DAG<T>::nbImplicitAnd % 1000000 == 0)
      printf("%dM ImplicitAnd size: %dMb\n", (int) DAG<T>::nbImplicitAnd/1000000,
             (int) sizeof(ImplicitAnd)*DAG<T>::nbImplicitAnd/1000000);        
  }

  DAG<T> *duplicateSameVarUnderAssumption_(int szAssums){assert(0); return NULL;}
      
  void toNNF(std::ostream& out) {
    const int index = DAG<T>::nodeToIndex[this];
    int nbUnitLit = b.nbUnit();
    for (int i = 0; i < nbUnitLit; i++) {
      out << "L " << readableLit((&DAG<T>::unitLits[b.idxUnitLit])[i]) << std::endl;
    }
    out << "A " << (nbUnitLit + 1) << " ";
    for (int i = 0; i < nbUnitLit; i++) {
      out << (index - i - 1) << " ";
    }
    out << DAG<T>::nodeToIndex[b.d] << std::endl;
  }

  // TODO: this is plain wrong but we don't really care as this will never be called.
  inline void printNNF(std::ostream& out){ assert(0);}// printNNF            

  bool isSAT(){assert(false); return false;}
  bool isSAT(vec<Lit> &unitsLit){assert(false); return false;}    
  T computeNbModels() {assert(false); return 0;}

private:
  Branch<T>& b;
};

#endif
