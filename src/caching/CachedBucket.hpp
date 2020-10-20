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
