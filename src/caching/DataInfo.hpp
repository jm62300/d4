/*
* d4
* Copyright (C) 2020  Univ. Artois & CNRS
* 
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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
  //   We always at least have the following distribution:
  // For info1 => |free (43 bytes)|nb var (21 bytes)|
  // For info2 => |szData (26 bytes)|nb octets data (2 bytes)|nb octets var (2 bytes)| free (2 bytes)|
  uint64_t info1;
  uint32_t info2;
  
  struct
  {
    unsigned count:29;    
    unsigned dirty:2;
    unsigned smudge:1;
  } stats;


 public:
  DataInfo();
  DataInfo(unsigned szData,
           unsigned nbVar,
           unsigned nbOctetsData,   
           unsigned nbOctetsVar,
           unsigned count);  

  inline bool smudge(){return stats.smudge;}
  inline void smudge(bool b){stats.smudge = b;}
  
  inline void reinitCount(int v = 0) {stats.count = v;}
  inline void incCount(int v = 1){stats.count += v;}
  inline void divCount(){stats.count >>= 1;}
  inline void decCount(int v = 1)
  {
    if(v > stats.count) stats.count = 0; else stats.count -= v;
  }
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
  inline void szData(unsigned sz)
  {
    info2 = (info2&((1<<6) - 1)) | ((uint32_t) sz << 6);
  }
  inline unsigned nbOctetsVar(){return 1 + ((info2>>2) & ((1<<2) - 1));}
  inline unsigned nbOctetsData(){return 1 + ((info2>>4) & ((1<<2) - 1));}
  inline unsigned nbVar(){return info1 & ((1<<21) - 1);}

  
  virtual void print(char *data, std::ostream &out)
  {
    out << data;
  }

  template <typename U> void printData(void *data, int sz, std::ostream &out)
  {
    U *p = static_cast<U *>(data);
    for(int i = 0 ; i<sz ; i++)
    {
      out << (unsigned) *p << " ";
      p++;
    }
    out << "\n";
  }// printdata
};
}

#endif
