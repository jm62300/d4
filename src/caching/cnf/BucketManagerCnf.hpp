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

#ifndef d4_src_caching_cnf_BucketManagerCnf_hpp
#define d4_src_caching_cnf_BucketManagerCnf_hpp

#include <bitset>
#include <vector>
#include <iostream>

#include "../../problem/cnf/CnfOccurrenceManager.hpp"
#include "../../problem/ProblemTypes.hpp"

#include "../BucketManager.hpp"
#include "../CachedBucket.hpp"


#define ALL 0
#define NB 1
#define NT 2


#define BIT_VECTOR 1
#define ONE_OCTET 2
#define TWO_OCTET 3
#define FOUR_OCTET 4
#define EIGHT_OCTET 5
#define MASK_REPR 7
#define DEC_SIZE 3

#define MASK 16383
#define MASK_HEADER 1048575

namespace d4
{
template<class T> class BucketManagerCnf : public BucketManager<T>
{
 protected:
  CnfOccurrenceManager *occManager;

  int modeStore;
  int nbClauseCnf;
  int nbVarCnf;
  int strategyCache;

 public:
  /**
     Constructor.

     @param[in] occM, the CNF occurrence manager
     @param[in] mdStore, the storing mode for the clause
     @param[in] strCache, the strategy used for caching
  */
  BucketManagerCnf(CnfOccurrenceManager *occM, int mdStore, int strCache) 
  {
    occManager = occM;
    assert(occM);    
    modeStore = mdStore;    
    strategyCache = strCache;
    updateOccManager(occM->getNbClause(), occM->getNbVariable(), occM->getMaxSizeClause());    
  }// BucketManager

  
  virtual ~BucketManagerCnf() {;}

  virtual void updateOccManager(int nbClause, int nbVar, int maxSizeClause) = 0;
  virtual void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) = 0;  
};

}

#endif
