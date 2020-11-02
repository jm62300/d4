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

#include <bitset>
#include <vector>
#include <iostream>

#include "src/caching/cnf/DataInfoCnf.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"
#include "src/problem/ProblemTypes.hpp"

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
template<class T> class BucketManager;

template<class T> class BucketManagerCnf : public BucketManager<T>
{  
 protected:
  SpecManagerCnf &specManager;
  
  int modeStore;
  unsigned nbClauseCnf;
  unsigned nbVarCnf;
  unsigned maxSizeClause;

 public:
  /**
     Constructor.

     @param[in] occM, the CNF occurrence manager
     @param[in] mdStore, the storing mode for the clause
     @param[in] strCache, the strategy used for caching
  */
  BucketManagerCnf(SpecManagerCnf &occM, int mdStore, unsigned sizePage) :
      specManager(occM)
  {
    modeStore = mdStore;
    nbClauseCnf = occM.getNbClause();
    nbVarCnf = occM.getNbVariable();
    maxSizeClause = occM.getMaxSizeClause();

    this->init(sizePage);
  }// BucketManager

  
  virtual ~BucketManagerCnf() {;}
  virtual void storeFormula(std::vector<Var> &component, CachedBucket<T> &b) = 0;  
};
} // d4
