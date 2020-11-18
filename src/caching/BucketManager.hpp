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
#pragma once

#include <iostream>
#include <cassert>
#include <vector>
#include <string.h>

#include <boost/program_options.hpp>

#include "src/problem/ProblemTypes.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/exceptions/FactoryException.hpp"

#include "CachedBucket.hpp"
#include "cnf/BucketManagerCnf.hpp"
#include "cnf/BucketManagerCnfCl.hpp"
#include "cnf/BucketManagerCnfIndex.hpp"

#define ONE_OCTET 2
#define TWO_OCTET 3
#define FOUR_OCTET 4
#define EIGHT_OCTET 5


namespace d4
{
namespace po = boost::program_options;

template<class T> class BucketManager
{
 protected:
  std::vector<char *> allocateData;
  char *data;
  unsigned long m_sizeFirstPage;
  unsigned long m_sizeAdditionalPage;
  unsigned long m_sizeData;
  unsigned long m_posInData;
  CachedBucket<T> bucket;
  
 public:
  // freespace[i][j] points to a free memory space of size i
  std::vector<std::vector<char *>> freeSpace;  
  unsigned long int allMemory;
  unsigned long int freeMemory;
  unsigned long int pageData;
  unsigned long int usedMemory;

  static BucketManager<T> *makeBucketManager(po::variables_map &vm,
                                             SpecManager &s,
                                             std::ostream &out)
  {
    std::string css = vm["cache-store-strategy"].as<std::string>();
    std::string ccr = vm["cache-clause-representation"].as<std::string>();
    unsigned long sizeFirstPage = vm["cache-size-first-page"].as<unsigned long>();
    unsigned long sizeAdditionalPage = vm["cache-size-additional-page"].as<unsigned long>();

    out << "c [CONSTRUCTOR] Cache bucket manager:"
        << " storage(" << css << ") "
        << " reprentation(" << ccr << ") "
        << " size_first_page(" << sizeFirstPage << ")"
        << " size_additional_page(" << sizeAdditionalPage << ")"
        << "\n";
    
    int modeStore = ALL;
    if(css == "not-binary") modeStore = NB;
    if(css == "not-touched") modeStore = NT;
    
    SpecManagerCnf &scnf = dynamic_cast<SpecManagerCnf&>(s);    
    if(ccr == "clause")
      return new BucketManagerCnfCl<T>(scnf, modeStore, sizeFirstPage, sizeAdditionalPage);
    if(ccr == "index")
      return new BucketManagerCnfIndex<T>(scnf, modeStore, sizeFirstPage, sizeAdditionalPage);

    throw (FactoryException("Cannot create a BucketManager",__FILE__, __LINE__));
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
    return ((double) freeMemory + (m_sizeData - m_posInData)) / (double) allMemory;
  } // remainingMemory


  /**
     Initialize the data structure regarding the configuration (ie. number of
     variables, maximum number of clauses and the lenght of the largest clause).

     @param[in] sizeFirstPage, the amount of bytes for the first page.
     @param[in] sizeAdditionalPage, the amount of bytes for the additional pages.
  */
  void init(unsigned long sizeFirstPage,
            unsigned long sizeAdditionalPage)
  {
    allMemory = freeMemory = m_posInData = 0;
    m_sizeFirstPage = sizeFirstPage;
    m_sizeAdditionalPage = sizeAdditionalPage;    
    m_sizeData = m_sizeFirstPage;

    // we cannot reinit ... at least for the moment
    assert(!allocateData.size());
    data = new char[m_sizeData];
    allocateData.push_back(data);
    allMemory += m_sizeData;
    usedMemory = 0;
  } // init


  /**
     Get a pointer on an available array where we can store the data we want to
     save into the bucket.
  */
  char *getArray(unsigned size)
  {
    usedMemory += size;
    char *ret = NULL;    

    if(size < freeSpace.size() && freeSpace[size].size())
    {
      ret = freeSpace[size].back();
      freeSpace[size].pop_back();
      freeMemory -= size;
      return ret;
    }
    
    // take a fresh entry
    if(m_posInData + size > m_sizeData)
    {
      unsigned rSz = m_sizeData - m_posInData;
      if(freeSpace.size() <= rSz) freeSpace.resize(rSz + 1, std::vector<char *>());
      freeSpace[rSz].push_back(&data[m_posInData]);
      freeMemory += rSz;

      printf("c Allocate a new page for the cache %lu\n", freeMemory);
      m_sizeData = m_sizeAdditionalPage;
      m_posInData = 0;
      data = new char[m_sizeData];
      allocateData.push_back(data);

      allMemory += m_sizeData;
    }
    
    ret = &data[m_posInData];
    m_posInData += size;
    return ret;
  } // getArray


  /**
     Release some memory of a given size and store this information in
     freespace.

     @param[in] m, the memory we want to release
     @param[in] size, the size of the memory block
  */
  inline void releaseMemory(char *m, unsigned size)
  {
    usedMemory -= size;
    
    if((m_posInData - size) > 0 && &data[m_posInData - size] == m)
      m_posInData -= size;
    else
    {
      if(size >= freeSpace.size()) freeSpace.resize(size + 1, std::vector<char *>());
      freeSpace[size].push_back(m);
      freeMemory += size;
    }
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
} // d4
