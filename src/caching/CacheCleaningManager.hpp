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

#include <boost/program_options.hpp>

#include "src/exceptions/FactoryException.hpp"

#include "CachedBucket.hpp"
#include "Cache.hpp"
#include "CacheCleaningNone.hpp"
#include "CacheCleaningCachet.hpp"
#include "CacheCleaningSharpSAT.hpp"
#include "CacheCleaningExpectation.hpp"

namespace d4
{
namespace po = boost::program_options;

template<class T> class CacheCleaningManager
{
 protected:
  Cache<T> *m_cache;

 public:
  virtual ~CacheCleaningManager(){}
  
  /**
     Create an operator to manage the cache reduction process.

     @param[in] vm, the option list.
     @param[in] cache, the cache where we want to clean.
     @param[in] nbVar, the number of variables in the problem.
     @param[in] out, the stream where are print out the information.
   */
  static CacheCleaningManager<T> *makeCacheCleaningManager(po::variables_map &vm,
                                                           Cache<T> *cache,
                                                           int nbVar,
                                                           std::ostream &out)
  {
    std::string crs = vm["cache-reduction-strategy"].as<std::string>();
    bool csa = vm["cache-smudge-activation"].as<bool>();

    if(crs == "cachet" || crs == "expectation")
    {
      if(crs == "cachet")
      {
        unsigned long limit = vm["cache-reduction-strategy-cachet-limit"].as<unsigned long>();
        out << "c [CONSTRUCTOR] Cache cleaning manager: " << crs
            << " smudge(" << csa << ") limit(" << limit << ")\n";
        return new CacheCleaningCachet<T>(cache, csa, limit);
      }
    
      if(crs == "expectation")
      {
        unsigned long limit = vm["cache-reduction-strategy-expectation-limit"].as<unsigned long>();
        double ratio = vm["cache-reduction-strategy-expectation-ratio"].as<double>();
        out << "c [CONSTRUCTOR] Cache cleaning manager: " << crs
            << " smudge(" << csa << ") limit(" << limit << ") "
            << " ratio(" << ratio << ")\n";
        return new CacheCleaningExpectation<T>(cache, csa, nbVar, limit, ratio); 
      }
    }else
    {
      out << "c [CONSTRUCTOR] Cache cleaning manager: " << crs << " smudge(" << csa << ")\n";
      if(crs == "none") return new CacheCleaningNone<T>(cache);
      if(crs == "sharpSAT") return new CacheCleaningSharpSAT<T>(cache, csa);
    }    
    
    throw (FactoryException("Cannot create a CacheCleaningManager",__FILE__, __LINE__));
  } // makeCacheCleaningManager
  

  virtual void initCountCachedBucket(CachedBucket<T> *cb) = 0;
  virtual void updateCountCachedBucket(CachedBucket<T> *cb, int nbVar) = 0;
  virtual void reduceCache() = 0;
  virtual void printCleaningInfo(std::ostream &out) = 0;


  /**
     Ask to the bucket manager to release some memory block.

     @param[in] data, the memory we release.
     @param[in] size, the size of the memory block.
     @param[in] posInHash, where in the cache this block oppears.
     @param[in] smudge, control if we want to directly remove the entry or not.
     
   */
  void releaseMemory(char *data, int size, int posInHash, bool smudge)
  {
    if(!smudge) m_cache->getBucketManager()->releaseMemory(data, size, -1);
    else m_cache->getBucketManager()->releaseMemory(data, size, posInHash);
  } // releaseMemory
  
};
  
} // d4
