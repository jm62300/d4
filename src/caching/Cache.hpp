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

#ifndef d4_src_caching_Cache_hpp
#define d4_src_caching_Cache_hpp

#define SIZE_HASH 999331

#include <vector>
#include <boost/program_options.hpp>

#include "CachedBucket.hpp"
#include <src/hashing/HashString.hpp>

namespace d4
{
namespace po = boost::program_options;
template<class T> class Cache
{
 private:
  bool verb;
  std::vector< std::vector< CachedBucket<T> > > hashTable;

  // statistics
  unsigned nbEntry;
  unsigned nbPositiveHit;
  unsigned nbNegativeHit;
  unsigned minAffectedHitCache;
  unsigned nbReduceCall;
  double sumAffectedHitCache;
  
  // data info
  unsigned nbInitVar;
  unsigned maxBlockClause;
  unsigned nbClauses;
  unsigned nbFailedInCache;
  unsigned nbRemoveEntry;
  unsigned long int nbCreationBucket;

  std::vector<int> sizeVarCacheHit;
  std::vector<int> nbCacheWithSizeVar;
  std::vector<int> nbTestCache;
  std::vector<bool> deadSize;

  unsigned maxSize;
  unsigned long int sumDataSize;

  unsigned strategyRedCache; // 0 no cache
  
  HashString hashMethod;
  
 public:
  Cache(po::variables_map &vm)
  {
    sumDataSize = nbEntry = nbCreationBucket = 0;
    nbPositiveHit = nbNegativeHit = 0;
    nbFailedInCache = 1;
    nbRemoveEntry = nbReduceCall = sumAffectedHitCache = 0;
    verb = 0;

    std::string strategyCacheOpt = vm["cache-reduction-strategy"].as<std::string>();
    if(strategyCacheOpt == "none") strategyRedCache = 0;
    else if(strategyCacheOpt == "expectation") strategyRedCache = 3;
    else assert(0);
  }// CacheCNF

  ~Cache()
  {
    hashTable.clear();
  }

  inline int getNbPositiveHit(){return nbPositiveHit;}
  inline int getNbNegativeHit(){return nbNegativeHit;}

  inline void printCacheInformation(std::ostream &out)
  {
    out << "c \033[1m\033[34mCache Information\033[0m\n";
    out << "c Number of positive hit: " << nbPositiveHit << "\n";
    out << "c Number of negative hit: " << nbNegativeHit << "\n";
    out << "c Number of reduceCall: " << nbReduceCall << "\n";
    out << "c\n";
  }// printCacheInformation


  
  inline void pushInHashTable(CachedBucket<T> &cb, unsigned int hashValue, T val)
  {
    hashTable[hashValue % SIZE_HASH].push_back(cb);

    CachedBucket<T> &cbIn = (hashTable[hashValue % SIZE_HASH].back());
    cbIn.lockedBucket(val);
    nbCreationBucket++;
    sumDataSize += cb.szData();
    
    switch(strategyRedCache)
    {
      case 0 : break;
      case 1 : cbIn.reinitCount(cb.nbVar()); break;
        // case 0 : cbIn.reinitCount(nbPositiveHit + nbNegativeHit); break;
      case 2 : ;
      case 3 : cbIn.reinitCount(nbPositiveHit + nbNegativeHit);
    }
    assert(cbIn.count());
    nbEntry++;
  }// pushinhashtable


  inline unsigned int computeHash(CachedBucket<T> &bucket)
  {
    return hashMethod.hash(bucket.data, bucket.szData());
  }
  
  /**
     Research in the set of buckets if the bucket pointed by i already exist.
     @param[in] idx, the index of the researched bucket
     \return the index of the identical bucket if this one exists, -1 otherwise
  */
  CachedBucket<T> *bucketAlreadyExist(CachedBucket<T> &cb, unsigned int hashValue)
  {
    char *refData = cb.data;
    std::vector<CachedBucket<T> > &listCollision = hashTable[hashValue % SIZE_HASH];

    for(int i = 0 ; i<listCollision.size() ; i++)
    {
      CachedBucket<T> &cbi = listCollision[i];
      if(!cb.sameHeader(cbi)) continue;

      if(!memcmp(refData, cbi.data, cbi.szData()))
      {
        if(!cbi.dirty()) sizeVarCacheHit[cbi.nbVar()]++;
        cbi.setTrueDirty();
        nbPositiveHit++;
        return &cbi;
      }
    }
    nbNegativeHit++;
    return NULL;
  }// bucketAlreadyExist


  /**
     Add an entry in the cache.
  */
  void addInCache(TmpEntry<T> &cb, T val)
  {
    pushInHashTable(cb.e, cb.hashValue, val);
  } // addInCache


  inline void callCleaningStrategy(BucketManager<T> *bm)
  {
    switch(strategyRedCache)
    {
      case 0 : break;
      case 1 : reduceCacheStr0(bm); break;
      case 2 : reduceCacheStr1(bm); break;
      case 3 : reduceCacheStr1(bm); break;
    }
  } // callCleaningStrategy


  /**
     Take a bucket manager as well as a set of variables consisting in the
     variables in the current component and search in the cache if the related
     formula is present in the cache, if it is not the case the bucket is
     created and added.

     @param[in] varConnected, the variable
     @param[in] bm, the bucket manager
  */
  TmpEntry<T> searchInCache(std::vector<Var> &varConnected, BucketManager<T> *bm)
  {
    if(strategyRedCache) callCleaningStrategy(bm);

    CachedBucket<T> *formulaBucket = bm->collectBuckect(varConnected);
    unsigned int hashValue = computeHash(*formulaBucket);
    CachedBucket<T> *cacheBucket = bucketAlreadyExist(*formulaBucket, hashValue);
    assert(nbTestCache.size() > varConnected.size());
    nbTestCache[varConnected.size()]++;

    if(cacheBucket)
    {
      switch(strategyRedCache)
      {
        case 2 : cacheBucket->reinitCount(nbPositiveHit + nbNegativeHit); break;
          // case 0 : cacheBucket->reinitCount(nbPositiveHit + nbNegativeHit); break;
        case 1 : cacheBucket->incCount(1); break;
      }
      bm->releaseMemory(formulaBucket->data, formulaBucket->szData());
      return TmpEntry<T>(*cacheBucket, hashValue, true);
    }
    else
    {
      nbFailedInCache++;
      nbCacheWithSizeVar[varConnected.size()]++;
      return TmpEntry<T>(*formulaBucket, hashValue, false);
    }
  } // searchInCache


  /**
     Create a bucket and store it in the cache.

     @param[in] varConnected, the variable
     @param[in] bm, the bucket manager
     @param[in] c, the value we want to store
  */
  inline void createAndStoreBucket(std::vector<Var> &varConnected,
                                   BucketManager<T> *bm, T &c)
  {
    CachedBucket<T> *formulaBucket = bm->collectBuckect(varConnected);
    unsigned int hashValue = computeHash(*formulaBucket);
    pushInHashTable(*formulaBucket, hashValue, c); // add the new bucket
    nbCacheWithSizeVar[varConnected.size()]++;
  }// createBucket


#define WINDOWS_CACHEDYN 30
#define START_CACHEDYN 10
  /**
     Compute a new limit from where we should not put the bucket in the cache.
  */
  inline int computeLimitVarCache()
  {
    double sumTest = 1, sumHit = 1;

    std::vector<int> compactNbTest, compactPosHit, sizeSet;
    for(int i = 1 ; i<sizeVarCacheHit.size() ; i++)
    {
      if(!nbTestCache[i]) continue;
      compactNbTest.push_back(nbTestCache[i]);
      compactPosHit.push_back(sizeVarCacheHit[i]);
      sizeSet.push_back(i);

      nbTestCache[i] >>= 1;
      if(sizeVarCacheHit[i]) sizeVarCacheHit[i] >>= 1;
    }

    int max = START_CACHEDYN < compactPosHit.size() ?
                               sizeSet[START_CACHEDYN] : sizeSet.back();
    for(int i = START_CACHEDYN ; i<compactPosHit.size() ; i++)
    {
      sumTest += compactNbTest[i];
      sumHit += compactPosHit[i];

      if(i >= WINDOWS_CACHEDYN + START_CACHEDYN)
      {
        sumTest -= compactNbTest[i - WINDOWS_CACHEDYN];
        sumHit -= compactPosHit[i - WINDOWS_CACHEDYN];
      }

      if(sumHit / sumTest > 1.0 / log2(sizeSet[i])) max = sizeSet[i];
    }


    if(max == sizeSet.back()) return max * 2;
    return max * 1.1;
  }// computeLimitVarCache


  inline bool shouldNotCache(int nbVar, BucketManager<T> *bm)
  {
    if(bm->allMemory < ((unsigned long int) 1<<30)) return false;
    if(deadSize[nbVar]) return true;
    deadSize[nbVar] = !sizeVarCacheHit[nbVar];

    if(deadSize[nbVar])
      printf("c entries s.t. |var| is of size %d are dead for caching\n", nbVar);
    return deadSize[nbVar];
  } // shouldNotCache


  /**
     Ckeck-out if we should postpone the creation of the current cache entry.
     @param[in] varConnected, the variables in the connected component
  */
  inline bool shouldPostpone(std::vector<Var> &varConnected)
  {
    return !nbCacheWithSizeVar[varConnected.size()];
  }// shouldPostpone


  /**
     Set the information concerning the number of clauses, variables
     and the maximum size of the clauses (these information are useful
     to know the size of the memory blocks we have to allocate).

     @param[in] mVar, the number of variables
     @param[in] nbC, the number of clauses
     @param[in] mSize, the size of the biggest clause
  */
  void setInfoFormula(unsigned int mVar, unsigned int nbC, int mSize)
  {
    minAffectedHitCache = mVar;
    maxSize = mSize;
    maxBlockClause = mSize * nbC;
    nbInitVar = mVar;

    for(int i = 0 ; i<nbInitVar ; i++)
    {
      sizeVarCacheHit.push_back(0);
      nbCacheWithSizeVar.push_back(0);
      nbTestCache.push_back(0);
      deadSize.push_back(false);
    }
  }// setInfoFormula


  /**
     Initialized the hashTable
  */
  void initHashTable(unsigned int mVar, unsigned int nbC, int mSize)
  {
    setInfoFormula(mVar, nbC, mSize);

    // init hash tables
    hashTable.clear();
    for(int i = 0 ; i<SIZE_HASH ; i++)
      hashTable.push_back(std::vector<CachedBucket<T> >());
  }// initHashTable


  /**
     Strategy 1 : Reduce the set of entries in the cache using the approach
     proposed by Muise.
  */
  void reduceCacheStr1(BucketManager<T> *bm)
  {
    if(!strategyRedCache) return;
    if(strategyRedCache == 2 && (bm->remainingMemory() > 0.1)) return;
    if(strategyRedCache == 3 && nbEntry < (10 * (1<<21))) return;

    nbReduceCall++;
    std::vector<int> vecCount;

    for(int i = 0 ; i<hashTable.size() ; i++)
    {
      std::vector< CachedBucket<T> > &v = hashTable[i];
      for(auto b : v) vecCount.push_back(b.count());
    }

    if(!vecCount.size()) return;
    std::sort(vecCount.begin(), vecCount.end());


    int limit = 0;
    if(strategyRedCache == 2) limit = vecCount[vecCount.size() >> 1];
    if(strategyRedCache == 3)
      limit = vecCount[(vecCount.size() >> 1) + (vecCount.size() >> 2)];

    for(int i = 0 ; i<hashTable.size() ; i++)
    {
      std::vector< CachedBucket<T> > &v = hashTable[i];

      for(int j = 0 ; j<v.size() ; )
      {
        CachedBucket<T> &cb = v[j];
        if(cb.count() < limit)
        {
          bm->releaseMemory(cb.data, cb.szData());
          v[j] = v.back();
          v.pop_back();
          nbRemoveEntry++;
          nbEntry--;
        } else
        {
          cb.divCount();
          j++;
        }
      }
    }

    std::cout << "c Call cache reduction: " << nbReduceCall << " " << nbRemoveEntry
              << " " << bm->freeMemory << "\n";
  } // reduceCacheStr1


  /**
     Remove from the cache structure the element that look to be useless (we use
     the dirty variable for this purpose).
  */
  void reduceCacheStr0(BucketManager<T> *bm)
  {
    if(!strategyRedCache) return;
    int dec = ((unsigned long int) nbEntry *
               (unsigned long int) (nbInitVar)) >>
              (38 - (int) log2(sumDataSize / nbCreationBucket));
    nbFailedInCache++;
    nbReduceCall++;

    for(int i = 0 ; i<hashTable.size() ; i++)
    {
      std::vector< CachedBucket<T> > &v = hashTable[i];

      for(int j = 0 ; j<v.size() ; )
      {
        CachedBucket<T> &cb = v[j];
        double ratio = (double) sizeVarCacheHit[cb.nbVar()] /
                       (double) nbCacheWithSizeVar[cb.nbVar()];
        bool mustBeKept = !deadSize[cb.nbVar()] &&
                          (cb.count() || cb.dirty() || ratio > 0.5);

        if(mustBeKept)
        {
          cb.decCount(dec);
          if(!cb.count() && cb.dirty())
          {
            cb.setFalseDirty();
            sizeVarCacheHit[cb.nbVar()]--;
          }
          j++;
        } else
        {
          if(cb.dirty()) sizeVarCacheHit[cb.nbVar()]--;
          nbCacheWithSizeVar[cb.nbVar()]--;

          bm->releaseMemory(cb.data, cb.szData());
          v[j] = v.back();
          v.pop_back();
          nbRemoveEntry++;
          nbEntry--;
        }
      }
    }

    printf("c Number of entries removed: %u %lu %lu %d %d %lu\n",
           nbRemoveEntry, bm->allMemory, bm->freeMemory, dec,
           nbEntry, sumDataSize / nbCreationBucket);
  } // reduceCacheStr0



  ///////////////////////////////////////////////////////////////////////////
  ////////////////  Show some information about the hash table  /////////////
  ///////////////////////////////////////////////////////////////////////////
  inline void showTabLit(unsigned long int *v, unsigned sz)
  {
    for(unsigned i = 0 ; i<sz ; i++) printf("%ld ", v[i]);
    printf("\n");
  } // showTabLit

  /**
     Function that give us the information about the size of the
     different conflict lists.
  */
  void showDistribution()
  {
    long int allC = 0;
    std::vector<int> countElt;
    for(int i = 0 ; i<hashTable.size() ; i++)
    {
      for(int j = 0 ; j<hashTable[i].size() ; j++)
      {
        countElt.push_back(hashTable[i][j].count());
        allC += hashTable[i][j].count();
      }
    }
    std::sort(countElt.begin(), countElt.end());

    int cpt = 1, val = 0, anotherSum = 0;
    for(int i = 0 ; i<countElt.size() ; i++)
    {
      if(countElt[i] == val) cpt++;
      else
      {
        anotherSum += cpt * val;
        printf("%d %d/%ld %d\n", cpt, val, allC, anotherSum);
        val = countElt[i];
        cpt = 1;
      }
    }
    printf("%d %d/%ld %d\n", cpt, val, allC, anotherSum);


    for(int i = 0 ; i<sizeVarCacheHit.size() ; i++)
    {
      if(sizeVarCacheHit[i]) printf("-> %d %d\n", i, sizeVarCacheHit[i]);
    }

    return;

    int biggestIdx = 0;
    std::vector<int> tabDistrib;
    for(int i = 0 ; i<hashTable.size() ; i++)
    {
      if(!hashTable[i].size()) continue;
      tabDistrib.push_back(hashTable[i].size());
      if(hashTable[biggestIdx].size() < hashTable[i].size()) biggestIdx = i;
    }

    std::sort(tabDistrib.begin(), tabDistrib.end());
    for(int i = 0 ; i<tabDistrib.size() ; i++) printf("%d ", tabDistrib[i]);
    printf("\n");

    float sum = 0;
    for(int i = 0 ; i<tabDistrib.size() ; i++) sum += tabDistrib[i];

    printf("average size of conflict list %lf %d\n",
           ((float) sum) / tabDistrib.size(), tabDistrib.size());
    printf("the median size is %d\n", tabDistrib[tabDistrib.size() >> 1]);
  }// showDistribution

};
} // d4

#endif
