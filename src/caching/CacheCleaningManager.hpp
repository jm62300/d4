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

namespace d4
{
namespace po = boost::program_options;

template<class T> class CacheCleaningManager
{
 protected:
  Cache<T> *m_cache;

 public:

  /**
     Create an operator to manage the cache reduction process.

     @param[in] vm, the option list.
     @param[in] cache, the cache where we want to clean.
   */
  static CacheCleaningManager<T> *makeCacheCleaningManager(po::variables_map &vm,
                                                           Cache<T> *cache,
                                                           std::ostream &out)
  {
    std::string crs = vm["cache-reduction-strategy"].as<std::string>();
    bool csa = vm["cache-smudge-activation"].as<bool>();

    out << "c [CONSTRUCTOR] Cache cleaning manager: " << crs << " smudge(" << csa << ")\n";
    
    if(crs == "none") return new CacheCleaningNone<T>(cache);
    if(crs == "cachet")
    {
      unsigned long limit = vm["cache-reduction-strategy-cachet-limit"].as<unsigned long>();
      return new CacheCleaningCachet<T>(cache, csa, limit);
    }
    if(crs == "sharpSAT") return new CacheCleaningSharSAT<T>(cache, csa);
    
    throw (FactoryException("Cannot create a CacheCleaningManager",__FILE__, __LINE__));
  } // makeCacheCleaningManager
  

  virtual void initCountCachedBucket(CachedBucket<T> &cb) = 0;
  virtual void updateCountCachedBucket(CachedBucket<T> &cb) = 0;
  virtual void reduceCache() = 0;
  virtual void printCleaningInfo(std::ostream &out) = 0;
  
};
  
} // d4
