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

#ifndef d4_src_caching_BucketManager_hpp
#define d4_src_caching_BucketManager_hpp

#include <iostream>
#include <cassert>
#include <vector>
#include <string.h>

#include <boost/program_options.hpp>

#include <src/problem/ProblemTypes.hpp>
#include <src/specs/SpecManager.hpp>

#include "CachedBucket.hpp"
#include "cnf/BucketManagerCnf.hpp"
#include "cnf/BucketManagerCnfCl.hpp"

#define ONE_OCTET 2
#define TWO_OCTET 3
#define FOUR_OCTET 4
#define EIGHT_OCTET 5

#define PAGE (1<<29)

namespace d4
{
namespace po = boost::program_options;

template<class T> class BucketManager
{
 protected:
  std::vector<char *> allocateData;
  char *data;
  unsigned long int sizeData, posInData;
  CachedBucket<T> bucket;
  
 public:
  // freespace[i][j] points to a free memory space of size i
  std::vector<std::vector<char *>> freeSpace;  
  unsigned long int allMemory;
  unsigned long int freeMemory;
  unsigned long int pageData;

  static BucketManager<T> *makeBucketManager(po::variables_map &vm,
                                             SpecManager &s)
  {
    std::string css = vm["cache-store-strategy"].as<std::string>();
    std::string ccr = vm["cache-clause-representation"].as<std::string>();
    unsigned sizePage = vm["cache-size-page"].as<unsigned>();

    int modeStore = ALL;
    if(css == "NB") modeStore = NB;
    if(css == "NT") modeStore = NT;
    
    SpecManagerCnf &scnf = dynamic_cast<SpecManagerCnf&>(s);    
    if(css == "clause") return new BucketManagerCnfCl<T>(scnf, modeStore, sizePage);
    
    return NULL;
  } // makeBucketManager

  
  virtual ~BucketManager()
  {
    for(unsigned i = 0 ; i<allocateData.size() ; i++) delete[](allocateData[i]);
    allocateData.clear();
  }

  inline int nbOctetToEncodeInt(unsigned int v) 
  {
    // we know that we cannot have more than 1<<32 variables
    if(v < (1<<8)) return 1;
    if(v < (1<<16)) return 2;
    return 4;
  }// nbOctetToEncodeInt


  inline double remainingMemory()
  {
    return ((double) freeMemory + (sizeData - posInData)) / (double) sizeData;
  } // remainingMemory


  /**
     Initialize the data structure regarding the configuration (ie. number of
     variables, maximum number of clauses and the lenght of the largest clause).
  */
  void init(int _sizeData)
  {
    allMemory = freeMemory = posInData = 0;
    sizeData = _sizeData;

    // we cannot reinit ... at least for the moment
    assert(!allocateData.size());
    data = new char[sizeData];
    allocateData.push_back(data);
    allMemory += sizeData;
  } // init


  /**
     Get a pointer on an available array where we can store the data we want to
     save into the bucket.
  */
  char *getArray(unsigned size)
  {
    char *ret = NULL;

    if(size < freeSpace.size() && freeSpace[size].size())
    {
      ret = freeSpace[size].back();
      freeSpace[size].pop_back();
      freeMemory -= size;
      return ret;
    }

    // go futher to see if we cannot split some entry
    if((size << 1) < freeSpace.size())
    {
      unsigned pos = size << 1;
      while(pos < freeSpace.size() && !freeSpace[pos].size()) pos++;

      // split an entry
      if(pos < freeSpace.size())
      {
        ret = freeSpace[pos].back();
        freeSpace[pos].pop_back();
        freeMemory -= size;
        freeSpace[pos - size].push_back(&ret[size]); // split and push
        return ret;
      }
    }

    // take a fresh entry
    if(posInData + size > sizeData)
    {
      unsigned rSz = sizeData - posInData;
      if(freeSpace.size() <= rSz) freeSpace.resize(rSz, std::vector<char *>());
      freeSpace[rSz].push_back(&data[posInData]);
      freeMemory += rSz;

      printf("c Allocate a new page for the cache %lu\n", freeMemory);
      posInData = 0;
      data = new char[sizeData];
      allocateData.push_back(data);

      allMemory += sizeData;
    }

    ret = &data[posInData];
    posInData += size;
    return ret;
  } // getArray


  /**
     Release some memory of a given size and store this information in
     freespace.

     @param[in] m, the memory we want to release
     @param[in] size, the size of the memory block
  */
  inline void releaseMemory(char *m, int size)
  {
    while(freeSpace.size() <= size) freeSpace.push_back(std::vector<char *>());
    freeSpace[size].push_back(m);
    freeMemory += size;
  }// reverseLastBucket


  /**
     Collect the bucket associtated to the set of variable given in
     parameter.
     @param[in] component, the variable belonging to the connected component
     \return a formula put in a bucket
  */
  CachedBucket<T> *collectBucket(std::vector<Var> &component)
  {
    storeFormula(component, bucket);
    return &bucket;
  }// collectBuckect


  virtual void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) = 0;
};
}


#endif
