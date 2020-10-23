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

#ifndef d4_src_heuristics_cnf_PartitioningHeuristicBipartite_hpp
#define d4_src_heuristics_cnf_PartitioningHeuristicBipartite_hpp

#include <vector>

#include <src/solvers/WrapperSolver.hpp>
#include <src/utils/EquivExtractor.hpp>
#include <src/specs/cnf/SpecManagerCnf.hpp>
#include "../PartitioningHeuristic.hpp"

namespace d4
{
class PartitioningHeuristicBipartite : public PartitioningHeuristic
{
 private:
  WrapperSolver &s;  
  SpecManagerCnf &om;
  EquivExtractor em;

  std::vector<bool> inCurrentComponent;
  std::vector<bool> markedVar;
  std::vector<bool> useLessVariable;
  std::vector<bool> isInput;
  std::vector<bool> markedClauses;
  std::vector<int> mapVar;
  std::vector<int> weightClause;
  std::vector<int> idxClauses;

  int *xpins;
  int *pins;
  int *cwghts;
  int *vwghts;
  int *partvec;
  int *partweights;
  
  int sumSize;
  int nbVar;
  int nbClause;

  // options
  bool reduceFormula; // 0.....01 
  bool equivSimp;     // 0.....10
  
 public:
  PartitioningHeuristicBipartite(WrapperSolver &s,
                                 SpecManager &om,
                                 unsigned options);

  PartitioningHeuristicBipartite(WrapperSolver &s,
                                 SpecManager &om,
                                 unsigned options,
                                 int nbClause,
                                 int nbVar,
                                 int sumSize);

  
  ~PartitioningHeuristicBipartite();
  
  void computePartition(std::vector<Var> &component,
                        std::vector<Var> &cutVar);

  void extractCutFromClauses(
      std::vector< std::vector<int> > &hyperEdges,
      std::vector<int> &cutSet,
      int *pv);
  
  void buildOccMap(std::vector< std::vector<int> > &hyperEdges,
                   std::vector<int> &idxClauses);

  void clearSetOfVariable(std::vector<Var> &component,
                          std::vector< std::vector<int> > &hypergraph,
                          std::vector<Var> &useFulVariable);

  void computeUselessVariables(std::vector<Var> &component,
                               std::vector< std::vector<int> > &hypergraph,
                               std::vector<int> &idxClauses);

  void computeUselessClauses(
      std::vector<int> &idxClauses,
      std::vector< std::vector<int> > &hypergraph);
};
} // d4

#endif
