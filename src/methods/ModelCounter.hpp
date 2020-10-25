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

#include <iostream>
#include <iomanip>
#include <ctime>
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
#include <src/utils/MemoryStat.hpp>

#include "MethodManager.hpp"

#define NB_SEP_MC 131
#define MASK_SHOWRUN_MC ((2<<13) - 1)
#define WIDTH_PRINT_COLUMN_MC 12


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
  unsigned nbDecisionNode;
  unsigned optCached;
  unsigned stampIdx;
  
  std::clock_t currentTime;

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
  BucketManager<T> *bucketManager;
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

    cache = new Cache<T>(vm);
    bucketManager = BucketManager<T>::makeBucketManager(vm, *specs);
    
    // we delete the useless objects.
    delete initProblem;
    delete preproc;

    // other variable initialization.
    currentTime = clock();

    // weight
    weightLit.resize((specs->getNbVariable() + 1) << 1, 1);
    weightVar.resize(specs->getNbVariable(), 2);
    
    optCached = vm["cache-activated"].as<bool>();
    callPartitioner = callEquiv = 0;
    nbSplit = nbCallCall = 0;    
    nbDecisionNode = nbNodeInCall = 0;

    stampIdx = 0;
    stampVar.resize(specs->getNbVariable(), 0);    
  } // constructor


  /**
     Destructor.
   */
  ~ModelCounter()
  {
    
  } // destructor


  /**
     Compute the current priority set.

     @param[in] connected, the current component.
     @param[in] priorityVar, the current priority variables.
     @param[out] currPriority, the intersection of the two previous sets.
  */
  inline void computePrioritySubSet(std::vector<Var> &connected,
                                    std::vector<Var> &priorityVar,
                                    std::vector<Var> &currPriority)
  {
    currPriority.clear();
    stampIdx++;
    for(auto &v : connected) stampVar[v] = stampIdx;
    for(auto &v : priorityVar)
      if(stampVar[v] == stampIdx && !specs->varIsAssigned(v))
        currPriority.push_back(v);
  } // computePrioritySet


  /**
     Print out information about the solving process.
     
     @param[in] out, the stream we use to print out information.
  */
  inline void showInter(std::ostream &out)
  {
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << nbCallCall 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << clock() - currentTime 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << cache->getNbPositiveHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << cache->getNbNegativeHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << nbSplit
        << std::fixed << std::setprecision(2)
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << callEquiv
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << nbDecisionNode
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << callPartitioner
        << "|\n";
  } // showInter

  /**
     Print out a line of dashes.
     
     @param[in] out, the stream we use to print out information.
   */
  inline void separator(std::ostream &out)
  {
    out << "c ";
    for(int i = 0 ; i<NB_SEP_MC ; i++) out << "-";
    out << "\n";
  } // separator

  /**
     Print out the header information.
     
     @param[in] out, the stream we use to print out information.
  */
  inline void showHeader(std::ostream &out)
  {
    separator(out);
    out << "c "
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#compile"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "time" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#posHit" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#negHit" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#equivCall" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#dec. Node" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#cutter"
        << "|\n";
    separator(out);
  } // showHeader


  /**
     Print out information when it is requiered.
     
     @param[in] out, the stream we use to print out information.
   */
  inline void showRun(std::ostream &out)
  {
    if(!(nbCallCall & (MASK_HEADER))) showHeader(out);
    if(nbCallCall && !(nbCallCall & MASK_SHOWRUN_MC)) showInter(out);
  } // showRun


  /**
     Print out the final stat.
     
     @param[in] out, the stream we use to print out information.
   */
  inline void printFinalStats(std::ostream &out)
  {
    separator(out);
    out << "c\n";
    out << "c \033[1m\033[31mStatistics \033[0m\n";
    out << "c \033[33mCompilation Information\033[0m\n";
    out << "c Number of recursive call: " << nbCallCall << "\n";
    out << "c Number of split formula: " << nbSplit << "\n";
    out << "c Number of decision: " << nbDecisionNode << "\n";
    out << "c Number of paritioner calls: " << callPartitioner << "\n";
    out << "c\n";
    cache->printCacheInformation(out);
    out << "c Final time: " << currentTime - clock() << "\n";
    out << "c\n";
  } // printFinalStat


  /**
     Initialize the assumption in order to compute the number of model
     under this one.

     @param[in] assums, the assumption
  */
  inline void initAssumption(std::vector<Lit> &assums)
  {
    solver->restart();
    solver->setAssumption(assums);
  } // initAssumption


  /**
     Return the weight of a given variable.

     @param[in] v, the variable.

     \return weightVar[v], that is the weight of v     
   */
  inline T getWeightVar(Var v)
  {
    return T(weightVar[v]);
  }

  /**
     Compute the value for free and unit variables.

     @param[in] units, the units variables
     @param[in] frees, the free variables

     \return the right value
  */
  inline T computeWeightUnitFree(std::vector<Lit> &units,
                                 std::vector<Var> &frees)
  {
    T tmp = 1;
    for(auto &l : units) tmp *= T(weightLit[l.intern()]);
    for(auto &v : frees) tmp *= T(weightVar[v]);
    return tmp;
  } // computeValue
  

  /**
     Call the CNF formula into a D-FPiBDD.

     @param[in] setOfVar, the current set of considered variables
     @param[in] unitsLit, the set of unit literal detected at this level
     @param[in] freeVariable, the variables which become free
     @param[in] priority, select in priority these variable to the next decision
     node
     @param[in] out, the stream we use to print out information.

     \return a number of models.
  */
  T computeNbModel_(std::vector<Var> &setOfVar,
                    std::vector<Lit> &unitsLit,
                    std::vector<Var> &freeVariable,
                    std::vector<Var> &priorityVar,
                    std::ostream &out)
  {
    showRun(out); nbCallCall++;
    
    solver->inputVar(setOfVar);
    if(!solver->solve()) return 0;
    
    solver->whichAreUnits(setOfVar, unitsLit); // collect unit literals
    specs->preUpdate(unitsLit);

    T ret = 1, curr;
    
    // compute the connected composant
    std::vector<Var> reallyPresent;
    std::vector< std::vector<Var> > varConnected;
    int nbComponent = specs->computeConnectedComponent(
        varConnected, setOfVar, freeVariable, reallyPresent);
    // consider each connected component.
    if(nbComponent)
    {
      nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for(int cp = 0 ; cp<nbComponent ; cp++)
      {
        std::vector<Var> &connected = varConnected[cp];
        specs->updateCurrentFormula(connected);
        TmpEntry<T> cb = optCached ?
                         cache->searchInCache(connected, bucketManager):
                         NULL_CACHE_ENTRY;

        if(optCached && cb.defined) ret *= cb.getValue();
        else
        {
          // recursive call
          std::vector<Var> currPriority;
          computePrioritySubSet(connected, priorityVar, currPriority);
          ret *= (curr = computeDecisionNode(connected, currPriority, out));

          if(optCached) cache->addInCache(cb, curr);
        }
        specs->popPreviousFormula();
      }
    }// else we have a tautology

    specs->postUpdate(unitsLit);    
    return ret;
  }// computeNbModel_


  /**
     This function select a variable and compile a decision node.

     @param[in] connected, the set of variable present in the current problem.
     @param[in] priorityVar, a list of variable we want to branch first.
     
     \return the compiled formula.
  */
  T computeDecisionNode(std::vector<Var> &connected,
                        std::vector<Var> &priorityVar,
                        std::ostream &out)
  {
    if(!priorityVar.size() && connected.size() > 10 && connected.size() < 5000)
      {
        heuristicPartition->computePartition(connected, priorityVar);
        assert(priorityVar.size());
        callPartitioner++;
      }

    // search the next variable to branch on
    std::vector<Var> &inVars = (priorityVar.size()) ? priorityVar : connected;    
    Var v = heuristicVar->selectVariable(inVars, *specs);    
    if(v == var_Undef) return 1;

    
    Lit l = Lit(v, heuristicPhase->selectPhase(v));
    nbDecisionNode++;
    
    // compile the formula where l is assigned to true
    std::vector<Lit> unitLitPos, unitLitNeg;
    std::vector<Var> freeVarPos, freeVarNeg;

    solver->pushAssumption(l);
    T pos = computeNbModel_(connected, unitLitPos, freeVarPos, priorityVar, out);
    pos *= computeWeightUnitFree(unitLitPos, freeVarPos);
    solver->popAssumption();
    
    solver->pushAssumption(~l);
    T neg = computeNbModel_(connected, unitLitNeg, freeVarNeg, priorityVar, out);
    neg *= computeWeightUnitFree(unitLitNeg, freeVarNeg);
    solver->popAssumption();
    
    return neg + pos;
  }// computeDecisionNode

  
  /**
     Compute the number of model using the trace of a SAT solver.

     \return the number of models
  */
  T computeNbModel(std::ostream &out)
  {
    std::vector<Var> freeVariable, setOfVar, priorityVar;
    std::vector<Lit> unitsLit;

    for(int i = 1 ; i <= specs->getNbVariable() ; i++) setOfVar.push_back(i);
    T d = computeNbModel_(setOfVar, unitsLit, freeVariable, priorityVar, out);

    T computeWeight = 1;
    for(auto &v : freeVariable) computeWeight *= T(weightVar[v]);
    for(auto &l : unitsLit) computeWeight *= T(weightLit[l.intern()]);

    return d * computeWeight;
  }// computeNbModel

 public:
  
  /**
     The method called to run the model counter.
   */
  void run()
  {
    T nbModels = computeNbModel(std::cout);
    std::cout << nbModels << "\n"; 
  } // run
  
};
} // d4

#endif

