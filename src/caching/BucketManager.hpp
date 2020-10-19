#ifndef d4_src_caching_BucketManager_hpp
#define d4_src_caching_BucketManager_hpp

#include <iostream>
#include <cassert>
#include <vector>
#include <string.h>

#include "../problem/ProblemTypes.hpp"
#include "CachedBucket.hpp"


#define ONE_OCTET 2
#define TWO_OCTET 3
#define FOUR_OCTET 4
#define EIGHT_OCTET 5

#define PAGE (1<<29)

namespace d4
{
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

  virtual void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) = 0;

  virtual ~BucketManager()
  {
    for(int i = 0 ; i<allocateData.size() ; i++) delete[](allocateData[i]);
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
  char *getArray(int size)
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
      int pos = size << 1;
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
      int rSz = sizeData - posInData;
      while(freeSpace.size() <= rSz) freeSpace.push_back();
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
};
}


#endif
