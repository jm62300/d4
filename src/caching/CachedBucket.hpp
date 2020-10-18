#ifndef d4_src_caching_CachedBucket_hpp
#define d4_src_caching_CachedBucket_hpp

#include "DataInfo.hpp"

namespace d4
{
template <class T> class CachedBucket
{
public:
  char *data;
  DataInfo header;
  T fc;

  CachedBucket() {data = NULL; header.szData(0);}

  inline void set(char *d, DataInfo &dnew)
  {
    data = d;
    header = dnew;
  }

  inline void lockedBucket(T v)
  {
    header.reinitCount();
    fc = v;
  }

  inline void reinitCount(int v = 0) {header.reinitCount(v);}
  inline void incCount(int v){header.incCount(v);}
  inline void divCount(){header.divCount();}
  inline void decCount(int v){header.decCount(v);}
  inline int count() {return header.count();}

  inline void reinitDirty() {header.reinitDirty();}
  inline void setTrueDirty(){header.setTrueDirty();}
  inline void incDirty(){header.incDirty();}
  inline void decDirty(){header.decDirty();}
  inline void setFalseDirty(){header.setFalseDirty();}
  inline int dirty() {return header.dirty();}

  inline void szData(int s) {header.szData(s);}
  inline int szData() {return header.szData();}
  inline int nbVar(){return header.nbVar();}
  inline void print(std::ostream &os){header.print(data, os);}
  
  inline bool sameHeader(CacheBucket<T> &b)
  {
    return header == b.header;
  }
};

}

#endif
