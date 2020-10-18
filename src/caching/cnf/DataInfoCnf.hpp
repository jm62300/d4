#ifndef d4_src_caching_cnf_DataInfoCnf_hpp
#define d4_src_caching_cnf_DataInfoCnf_hpp

#include "../DataInfo.hpp"

namespace d4
{
class DataInfoCnf : public DataInfo
{
  void print(char *data, std::ostream &out);

  unsigned nbOctetsDistrib(){return info2 & ((1<<2) - 1);}
  unsigned nbClause(){return info1>>42 & ((1<<22) - 1);}
  unsigned nbLit(){return info1>>21 & ((1<<21) - 1);}

  // need to be computed
  unsigned nbDiffSize()
  {
    if(!nbOctetsDistrib()) return 0;
    return (szData() - nbLit() * nbOctetsData() - nbVar() * nbOctetsVar()) / nbOctetsDistrib();
  }
};
}

#endif
