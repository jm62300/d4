#ifndef Minisat_DAG_DecompAndNodeCertified_h
#define Minisat_DAG_DecompAndNodeCertified_h

#include <string>
#include "DAG.hh"

#define BLOCK_ALLOC_ALL_CHILDREN 1<<20

template<class T> class DAG;
template<class T> class DecomposableAndNodeCertified : public DAG<T>
{
public:
  using DAG<T>::nbEdges;
  using DAG<T>::saveFreeVar;
  using DAG<T>::globalStamp;
  using DAG<T>::idxOutputStruct;
  using DAG<T>::stamp;

  static DAG<T> **allChildren;
  static int capSzAllChildren, szAllChildren;
  vec<bool> comeFromCache;

  struct
  {
    unsigned szChildren:22;
    unsigned posInAllChildren:32;
  } header;

  DecomposableAndNodeCertified(vec<DAG<T> *> &sons, vec<bool> &comeFromCache_)
  {
    comeFromCache_.copyTo(comeFromCache);
    header.szChildren = sons.size();
    header.posInAllChildren = giveMeEmplacementChildren(sons.size());

    DAG<T> **children = &allChildren[header.posInAllChildren];
    for(int i = 0 ; i<header.szChildren ; i++) children[i] = sons[i];

    nbEdges += header.szChildren;
    szAllChildren += sons.size();
  }

  ~DecomposableAndNodeCertified()
  {
    szAllChildren -= header.szChildren;
    if(!szAllChildren)
      {
        free(allChildren);
        allChildren = NULL;
      }
  }

  inline unsigned int giveMeEmplacementChildren(int nbChildren)
  {
    while(capSzAllChildren < (szAllChildren + nbChildren))
    {
      capSzAllChildren += BLOCK_ALLOC_ALL_CHILDREN;
      allChildren = (DAG<T> **) realloc(allChildren, capSzAllChildren * sizeof(DAG<T>*));
      if(!allChildren)
      {
        printf("Memory out bufferInfo %d\n",(int) (capSzAllChildren * sizeof(DAG<T>)));
        exit(30);
      }
    }
    return szAllChildren;
  }// giveMeEmplacementChildren


  inline int getSize_()
  {
    int cpt = 0;
    DAG<T> **children = &allChildren[header.posInAllChildren];
    for(int i = 0 ; i<header.szChildren ; i++) cpt += children[i]->getSize_();
    return 1 + cpt;
  }


  inline void printNNF(std::ostream& out, bool certif)
  {
    if(stamp >= globalStamp) return;
    stamp = globalStamp + idxOutputStruct + 1;
    int idxCurrent = ++idxOutputStruct;

    out << "a " << idxCurrent << " " << header.szChildren << " 0" << endl;
    DAG<T> **children = &allChildren[header.posInAllChildren];
    for(int i = 0 ; i<header.szChildren ; i++)
      {
        children[i]->printNNF(out, certif);
        out << idxCurrent << " " << children[i]->getIdx() << (comeFromCache[i] ? " 1" : " 2") << " 0\n";
      }
  }// printNNF


  inline bool isSAT(vec<Lit> &unitsLitBranches)
  {
    DAG<T> **children = &allChildren[header.posInAllChildren];
    for(int i = 0 ; i < header.szChildren ; i++) if(!children[i]->isSAT(unitsLitBranches)) return false;
    return true;
  }


  inline T computeNbModels()
  {
    T nbModels = 1;
    DAG<T> **children = &allChildren[header.posInAllChildren];
    for(int i = 0 ; i < header.szChildren ; i++) nbModels *= children[i]->computeNbModels();
    return nbModels;
  }// computeNbModels
};

// initialize the static attributs
template<typename T> int DecomposableAndNodeCertified<T>::capSzAllChildren{0};
template<typename T> int DecomposableAndNodeCertified<T>::szAllChildren{0};
template<typename T> DAG<T> **DecomposableAndNodeCertified<T>::allChildren{NULL};
#endif
