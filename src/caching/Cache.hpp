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

#define SIZE_HASH 999331

#include <vector>
#include <boost/program_options.hpp>

#include "src/hashing/HashString.hpp"
#include "src/specs/SpecManager.hpp"

#include "CacheCleaningManager.hpp"
#include "BucketManager.hpp"
#include "CachedBucket.hpp"

namespace d4
{
namespace po = boost::program_options;
template<class T> class Cache
{
 private:
  bool verb;
  std::vector< std::vector< CachedBucket<T> > > hashTable;

  // statistics
  unsigned long m_nbEntry;
  unsigned long m_nbPositiveHit;
  unsigned long m_nbNegativeHit;
  unsigned minAffectedHitCache;
  double sumAffectedHitCache;
  
  // data info
  unsigned nbInitVar;
  unsigned nbFailedInCache;
  unsigned nbRemoveEntry;
  unsigned long int m_nbCreationBucket;
  unsigned long int m_sumDataSize;

  std::ostream m_out;
  HashString hashMethod;
  BucketManager<T> *m_bucketManager;
  CacheCleaningManager<T> *m_cacheCleaningManager;
  
 public:
  Cache(po::variables_map &vm, unsigned nbVar,
        SpecManager *specs, std::ostream &out) : m_out(nullptr)
  {
    // init the output stream
    m_out.copyfmt(out);
    m_out.clear(out.rdstate());           
    m_out.basic_ios<char>::rdbuf(out.rdbuf());
    
    m_sumDataSize = m_nbEntry = m_nbCreationBucket = 0;
    m_nbPositiveHit = m_nbNegativeHit = 0;
    nbFailedInCache = 1;
    nbRemoveEntry = sumAffectedHitCache = 0;
    verb = 0;
    
    initHashTable(nbVar);
    m_cacheCleaningManager = CacheCleaningManager<T>::makeCacheCleaningManager(vm, this, nbVar, out);
    m_bucketManager = BucketManager<T>::makeBucketManager(vm, this, *specs, out);
  }// CacheCNF

  ~Cache()
  {
    hashTable.clear();
    delete m_cacheCleaningManager;
    delete m_bucketManager;
  }


  inline unsigned long int nbCreationBucket(){return m_nbCreationBucket;}
  inline unsigned long int sumDataSize(){return m_sumDataSize;}
  inline unsigned long int usedMemory(){return m_bucketManager->usedMemory;}
  inline unsigned long int getNbPositiveHit(){return m_nbPositiveHit;}
  inline unsigned long int getNbNegativeHit(){return m_nbNegativeHit;}
  inline unsigned long getNbEntry(){return m_nbEntry;}
  inline void decrementNbEntry(){m_nbEntry--;}
  inline BucketManager<T> *getBucketManager(){return m_bucketManager;}
  inline std::vector< std::vector< CachedBucket<T> > > &getHashTable(){return hashTable;}

  inline void printCacheInformation(std::ostream &out)
  {
    out << "c \033[1m\033[34mCache Information\033[0m\n";
    out << "c Number of positive hit: " << m_nbPositiveHit << "\n";
    out << "c Number of negative hit: " << m_nbNegativeHit << "\n";
    m_cacheCleaningManager->printCleaningInfo(out);    
    out << "c\n";
  }// printCacheInformation


  
  inline void pushInHashTable(CachedBucket<T> &cb, unsigned int hashValue, T val)
  {
    hashTable[hashValue % SIZE_HASH].push_back(cb);

    CachedBucket<T> &cbIn = (hashTable[hashValue % SIZE_HASH].back());
    cbIn.lockedBucket(val);
    m_nbCreationBucket++;
    m_sumDataSize += cb.szData();
    m_cacheCleaningManager->initCountCachedBucket(&cbIn);
    m_nbEntry++;
  }// pushinhashtable


  inline unsigned computeHash(CachedBucket<T> &bucket)
  {
    return hashMethod.hash(bucket.data, bucket.szData());
  }
  
  /**
     Research in the set of buckets if the bucket pointed by i already exist.
     @param[in] idx, the index of the researched bucket
     \return the index of the identical bucket if this one exists, NULL otherwise
  */
  CachedBucket<T> *bucketAlreadyExist(CachedBucket<T> &cb, unsigned hashValue)
  {
    char *refData = cb.data;
    std::vector<CachedBucket<T> > &listCollision = hashTable[hashValue % SIZE_HASH];

    for(auto &cbi : listCollision)
    {
      if(!cb.sameHeader(cbi)) continue;

      if(!memcmp(refData, cbi.data, cbi.szData()))
      {        
        m_nbPositiveHit++;
        return &cbi;
      }
    }
    
    m_nbNegativeHit++;
    return NULL;
  }// bucketAlreadyExist


  /**
     Add an entry in the cache.
  */
  void addInCache(TmpEntry<T> &cb, T val)
  {
    pushInHashTable(cb.e, cb.hashValue, val);
  } // addInCache


  /**
     Take a bucket manager as well as a set of variables consisting in the
     variables in the current component and search in the cache if the related
     formula is present in the cache, if it is not the case the bucket is
     created and added.

     @param[in] varConnected, the variable
  */
  TmpEntry<T> searchInCache(std::vector<Var> &varConnected)
  {
    m_cacheCleaningManager->reduceCache();

    CachedBucket<T> *formulaBucket = m_bucketManager->collectBucket(varConnected);
    unsigned hashValue = computeHash(*formulaBucket);
    CachedBucket<T> *cacheBucket = bucketAlreadyExist(*formulaBucket, hashValue);
    
    m_cacheCleaningManager->updateCountCachedBucket(cacheBucket, varConnected.size());
    if(!cacheBucket) return TmpEntry<T>(*formulaBucket, hashValue, false);
    
    m_bucketManager->releaseMemory(formulaBucket->data, formulaBucket->szData());
    return TmpEntry<T>(*cacheBucket, hashValue, true);
  } // searchInCache


  /**
     Create a bucket and store it in the cache.

     @param[in] varConnected, the variable
     @param[in] c, the value we want to store
  */
  inline void createAndStoreBucket(std::vector<Var> &varConnected, T &c)
  {
    CachedBucket<T> *formulaBucket = m_bucketManager->collectBuckect(varConnected);
    unsigned int hashValue = computeHash(*formulaBucket);
    pushInHashTable(*formulaBucket, hashValue, c); // add the new bucket
  }// createBucket


  /**
     Set the information concerning the number of clauses, variables
     and the maximum size of the clauses (these information are useful
     to know the size of the memory blocks we have to allocate).

     @param[in] mVar, the number of variables
  */
  void setInfoFormula(unsigned mVar)
  {
    minAffectedHitCache = mVar;
    nbInitVar = mVar;
  }// setInfoFormula


  /**
     Initialized the hashTable
  */
  void initHashTable(unsigned maxVar)
  {
    setInfoFormula(maxVar);

    // init hash tables
    hashTable.clear();
    hashTable.resize(SIZE_HASH, std::vector<CachedBucket<T> >());
  }// initHashTable


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
