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

#include <cstdint>
#include <vector>
#include <boost/program_options.hpp>

#include "src/solvers/WrapperSolver.hpp"
#include "src/utils/EquivExtractor.hpp"
#include "src/specs/cnf/SpecManagerCnf.hpp"
#include "src/partitioner/PartitionerManager.hpp"
#include "src/hyperGraph/HyperGraphExtractor.hpp"

#include "../PartitioningHeuristic.hpp"

namespace d4
{
namespace po = boost::program_options;
class PartitioningHeuristicStatic : public PartitioningHeuristic
{  
 protected:
  struct Strata
  {
    unsigned fatherId;
    std::vector<unsigned> part;
  };
  
  WrapperSolver &m_s;  
  SpecManagerCnf &m_om;
  EquivExtractor m_em;
  PartitionerManager *m_pm;
  
  unsigned m_nbVar;
  unsigned m_nbClause;
  unsigned m_maxNbNodes;
  unsigned m_maxNbEdges;

  bool m_isInitialized;
  std::vector<unsigned> m_bucketNumber;
  std::vector<unsigned> m_separtorLevel;

  // options:
  bool m_reduceFormula; 
  bool m_equivSimp;
  
 protected:
  virtual void computeDecomposition(
      std::vector<Var> &component,
      std::vector<Var> &equivClass,
      std::vector< std::vector<Var> > &equivVar,
      std::vector<unsigned> &bucketNumber) = 0;
  
  void init();
  
 protected:
  PartitioningHeuristicStatic(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om);

  PartitioningHeuristicStatic(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om,                                 
      int nbClause,
      int nbVar,
      int sumSize);

 public:  
  virtual ~PartitioningHeuristicStatic();

  inline std::vector<unsigned> &getBucketNumber(){return m_bucketNumber;}
  inline std::vector<unsigned> &getSeparatorLevel(){return m_separtorLevel;}
  
  static PartitioningHeuristicStatic *makePartitioningHeuristicStatic(
      po::variables_map &vm,
      WrapperSolver &s,
      SpecManager &om,
      int nbClause,
      int nbVar,
      int sumSize,
      const std::string &type);
  
  virtual void computeCutSet(
      std::vector<Var> &component,
      std::vector<Var> &cutSet) = 0;

  virtual bool isReady() {return true;}
};
} // d4
