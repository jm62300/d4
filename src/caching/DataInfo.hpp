#ifndef d4_src_caching_DataInfo_hpp
#define d4_src_caching_DataInfo_hpp

#include <iostream>
#include <cassert>

#include <stdio.h>
#include <cstdint>
#include <math.h>


namespace d4
{
class DataInfo
{
 protected:
  // we reserve 96 bytes to store information in the cached bucket
  uint64_t info1;
  uint32_t info2;
  
  struct
  {
    unsigned count:30;
    unsigned dirty:2;
  } stats;


 public:
  DataInfo();
  DataInfo(uint64_t i1, uint32_t i2, unsigned count);  

  inline void reinitCount(int v = 0) {stats.count = v;}
  inline void incCount(int v = 1){stats.count += v;}
  inline void divCount(){stats.count >>= 1;}
  inline void decCount(int v = 1){if(v > stats.count) stats.count = 0; else stats.count -= v;}
  inline int count() {return stats.count;}

  inline int dirty() {return stats.dirty;}
  inline void reinitDirty() {stats.dirty = 0;}
  inline void incDirty() { if(stats.dirty < 3) stats.dirty++;}
  inline void decDirty() { if(stats.dirty > 0) stats.dirty--;}
  inline void setTrueDirty(){stats.dirty = 1;}
  inline void setFalseDirty(){stats.dirty = 0;}

  bool operator==(const DataInfo& d)
  {
    return info1 == d.info1 && info2 == d.info2;
  } // operator ==

  virtual ~DataInfo(){}

  inline unsigned szData(){return info2 >> 6;}
  inline void szData(unsigned sz){info2 = (info2&((1<<6) - 1)) | ((uint32_t) sz << 6);}
  inline unsigned nbOctetsVar(){return (info2>>2) & ((1<<2) - 1);}
  inline unsigned nbOctetsData(){return (info2>>4) & ((1<<2) - 1);}
  inline unsigned nbVar(){return info1 & ((1<<21) - 1);}

  
  virtual void print(char *data, std::ostream &out)
  {
    out << data;
  }

  template <typename U> void printData(void *data, int sz)
  {
    U *p = static_cast<U *>(data);
    for(int i = 0 ; i<sz ; i++)
    {
      printf("%d ", *p);
      p++;
    }
    printf("\n");
  }// printdata
};
}

#endif
