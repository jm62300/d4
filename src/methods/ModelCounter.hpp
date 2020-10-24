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

#ifndef d4_src_methods_ModelCounter_hpp
#define d4_src_methods_ModelCounter_hpp

#include <boost/program_options.hpp>

#include <src/heuristics/ScoringMethod.hpp>
#include <src/heuristics/PhaseHeuristic.hpp>
#include <src/heuristics/PartitioningHeuristic.hpp>
#include <src/problem/ProblemManager.hpp>
#include <src/problem/ProblemTypes.hpp>
#include <src/preprocs/PreprocManager.hpp>
#include <src/specs/SpecManager.hpp>
#include <src/solvers/WrapperSolver.hpp>
#include <src/caching/BucketManager.hpp>
#include <src/caching/TmpEntry.hpp>
#include <src/caching/Cache.hpp>
#include <src/caching/CachedBucket.hpp>


#include "MethodManager.hpp"

namespace d4
{
namespace po = boost::program_options;
template <class T> class ModelCounter : public MethodManager
{
 private:
  bool optDomConst;
  bool optReversePolarity;
  
  unsigned nbNodeInCall;
  unsigned nbCallCall;
  unsigned nbSplit;
  unsigned callEquiv;
  unsigned callPartitioner;
  unsigned limitCacheDyn;
  unsigned nbDecisionNode;
  unsigned freqLimitDyn;
  unsigned optCached;
  unsigned stampIdx;
  
  double currentTime;

  std::vector<unsigned> stampVar;
  std::vector<double> weightLit;
  std::vector<double> weightVar;
  std::vector< std::vector<Lit> > clauses;

  ProblemManager *problem;
  WrapperSolver *solver;
  SpecManager *specs;
  ScoringMethod *heuristicVar;
  PhaseHeuristic *heuristicPhase;
  PartitioningHeuristic * heuristicPartition;
  BucketManager<T> *bm;
  TmpEntry<T> NULL_CACHE_ENTRY;  
  Cache<T> *cache;

 public:

  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  ModelCounter(po::variables_map &vm)
  {
    // the initial problem.
    ProblemManager *initProblem = ProblemManager::makeProblemManager(vm);
    assert(initProblem);

    // we call the preproc and we generate the problem used after.
    PreprocManager *preproc = PreprocManager::makePreprocManager(vm);
    assert(preproc);
    problem = preproc->run(*initProblem);
    assert(problem);

    // we create the SAT solver. 
    solver = WrapperSolver::makeWrapperSolver(vm);
    assert(solver);
    solver->initSolver(*problem);

    // we initialize the object that will give info about the problem.
    specs = SpecManager::makeSpecManager(vm, *problem);
    assert(specs);

    // we initialize the object used to compute score and partition.
    heuristicVar = ScoringMethod::makeScoringMethod(vm, *specs, *solver);    
    heuristicPhase = PhaseHeuristic::makePhaseHeuristic(vm, *specs, *solver);
    heuristicPartition = PartitioningHeuristic::
                         makePartitioningHeuristic(vm, *specs, *solver);
    assert(heuristicVar && heuristicPhase && heuristicPartition);
    
    // we delete the useless objects.
    delete initProblem;
    delete preproc;    
  } // constructor


  /**
     Destructor.
   */
  ~ModelCounter()
  {
    
  } // destructor

  /**
     The method called to run the model counter.
   */
  void run()
  {
    
  } // run
  
};
} // d4

#endif

