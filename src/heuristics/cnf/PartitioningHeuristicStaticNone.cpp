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
#include "PartitioningHeuristicStaticNone.hpp"

namespace d4
{

/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
 */
PartitioningHeuristicStaticNone::PartitioningHeuristicStaticNone(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om) :
    PartitioningHeuristicStaticNone(
        vm, s, om,
        dynamic_cast<SpecManagerCnf&>(om).getNbClause(),
        dynamic_cast<SpecManagerCnf&>(om).getNbVariable(),
        dynamic_cast<SpecManagerCnf&>(om).getSumSizeClauses())
{
  
} // constructor


/**
   Constructor.

   @param[in] vm, the option list.
   @param[in] s, a wrapper on a solver.
   @param[in] om, a structure manager.
   @param[in] nbClause, the number of clauses.
   @param[in] nbVar, the number of variables.
   @param[in] sumSize, which give the number of literals.
 */
PartitioningHeuristicStaticNone::PartitioningHeuristicStaticNone(
    po::variables_map &vm,
    WrapperSolver &s,
    SpecManager &om,                                 
    int nbClause,
    int nbVar,
    int sumSize) :
    PartitioningHeuristicStatic(vm, s, om, nbClause, nbVar, sumSize)
{
  std::cout << "c [CONSTRUCTOR] Static partitioner: none\n";

  m_isInitialized = true;
  m_bucketNumber.resize(nbVar + 1, 1);
} // constructor


/**
   Destructor.
 */
PartitioningHeuristicStaticNone::~PartitioningHeuristicStaticNone()
{
  
} // destructor

} // d4
