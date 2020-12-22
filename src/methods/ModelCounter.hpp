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

#include <iostream>
#include <iomanip>
#include <ctime>
#include <boost/program_options.hpp>

#include "src/heuristics/ScoringMethod.hpp"
#include "src/heuristics/PhaseHeuristic.hpp"
#include "src/heuristics/PartitioningHeuristic.hpp"
#include "src/problem/ProblemManager.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/preprocs/PreprocManager.hpp"
#include "src/specs/SpecManager.hpp"
#include "src/solvers/WrapperSolver.hpp"
#include "src/caching/TmpEntry.hpp"
#include "src/caching/Cache.hpp"
#include "src/caching/CachedBucket.hpp"
#include "src/utils/MemoryStat.hpp"

#include "MethodManager.hpp"

#define NB_SEP_MC 118
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
  unsigned callPartitioner;
  unsigned nbDecisionNode;
  unsigned optCached;
  unsigned stampIdx;
  unsigned m_limitUpdate;
  unsigned m_limitUpdateCounter;
  bool m_staticLimit;

  std::vector<unsigned> stampVar;
  std::vector< std::vector<Lit> > clauses;

  std::vector<unsigned long> nbTestCacheVarSize;
  std::vector<unsigned long> nbPosHitCacheVarSize;

  ProblemManager *problem;
  WrapperSolver *solver;
  SpecManager *specs;
  ScoringMethod *m_hVar;
  PhaseHeuristic *m_hPhase;
  PartitioningHeuristic *m_hCutSet;
  TmpEntry<T> NULL_CACHE_ENTRY;  
  Cache<T> *m_cache;

  std::ostream m_out;

  unsigned limitNbVarCache;
  double ratioDynamicLimit;
  unsigned limitNbVarCacheDynamic;
  
 public:

  /**
     Constructor.

     @param[in] vm, the list of options.
   */
  ModelCounter(po::variables_map &vm) : m_out(nullptr)
  {
    // init the output stream
    m_out.copyfmt(std::cout);                          
    m_out.clear(std::cout.rdstate());           
    m_out.basic_ios<char>::rdbuf(std::cout.rdbuf());
    
    // the initial problem.
    ProblemManager *initProblem = ProblemManager::makeProblemManager(vm, m_out);
    m_out << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input formula\033[0m\n";
    initProblem->displayStat(m_out, "c [INITIAL INPUT] ");
    m_out << "c\n";
    assert(initProblem);    

    // we call the preproc and we generate the problem used after.
    PreprocManager *preproc = PreprocManager::makePreprocManager(vm, m_out);
    assert(preproc);
    problem = preproc->run(*initProblem);
    m_out << "c [PREPROCESSED INPUT] \033[4m\033[32mStatistics about the preprocessed formula\033[0m\n";
    problem->displayStat(m_out, "c [PREPROCESSED INPUT] ");
    m_out << "c\n";
    assert(problem);

    // we create the SAT solver. 
    solver = WrapperSolver::makeWrapperSolver(vm, m_out);
    assert(solver);
    solver->initSolver(*problem);
    solver->setNeedModel(true);
    
    // we initialize the object that will give info about the problem.
    specs = SpecManager::makeSpecManager(vm, *problem, m_out);
    assert(specs);
    
    // we initialize the object used to compute score and partition.
    m_hVar = ScoringMethod::makeScoringMethod(vm, *specs, *solver, m_out);    
    m_hPhase = PhaseHeuristic::makePhaseHeuristic(vm, *specs, *solver, m_out);
    m_hCutSet = PartitioningHeuristic::makePartitioningHeuristic(vm, *specs, *solver, m_out);
    assert(m_hVar && m_hPhase && m_hCutSet);
    
    m_cache = new Cache<T>(vm, problem->getNbVar(), specs, m_out);
    
    // we delete the useless objects.
    delete initProblem;
    delete preproc;

    // init the clock time.
    initTimer();
    
    optCached = vm["cache-activated"].as<bool>();
    callPartitioner = 0;
    nbSplit = nbCallCall = 0;    
    nbDecisionNode = nbNodeInCall = 0;

    m_limitUpdate = vm["cache-limit-update-frequency"].as<unsigned>();
    m_staticLimit = vm["cache-limit-static"].as<bool>();
    m_limitUpdateCounter = 0;

    if(m_staticLimit)
    {
      ratioDynamicLimit = 1;
      limitNbVarCache = problem->getNbVar();
    }else
    {
      ratioDynamicLimit = vm["cache-limit-ratio-number-variable"].as<double>();    
      limitNbVarCache = vm["cache-limit-number-variable"].as<unsigned>();
    }
    limitNbVarCacheDynamic = limitNbVarCache;
    
    m_out << "c [CONSTRUCTOR] Limit number of variables for caching: "
          << "static("<< m_staticLimit << ") "
          << "limit("<< limitNbVarCache << ") "
          << "ratio("<< ratioDynamicLimit << ") "
          << "limitDyn("<< limitNbVarCacheDynamic << ") "
          << "freq update(" << m_limitUpdate << ") "
          << "\n";
    
    stampIdx = 0;
    stampVar.resize(specs->getNbVariable() + 1, 0);
    nbTestCacheVarSize.resize(specs->getNbVariable() + 1, 0);
    nbPosHitCacheVarSize.resize(specs->getNbVariable() + 1, 0);
    
    std::vector<Var> &selected = problem->getSelectedVar();
    for(auto v : selected) std::cout << v << " ";
    std::cout << "\n";    
  } // constructor


  /**
     Destructor.
   */
  ~ModelCounter()
  {
    delete problem;
    delete solver;
    delete specs;
    delete m_hVar;
    delete m_hPhase;
    delete m_hCutSet;
    delete m_cache;
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
        << std::fixed << std::setprecision(2)
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << getTimer()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->getNbPositiveHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->getNbNegativeHit()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << m_cache->usedMemory()
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << nbSplit
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << MemoryStat::memUsedPeak()
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
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "memory"
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#split" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "mem(MB)"
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
    m_cache->printCacheInformation(out);
    out << "c Final time: " << getTimer() << "\n";
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
     Decide if the cache is realized or not.
   */
  bool cacheIsActivated(std::vector<Var> &connected)
  {
    if(!optCached) return false;
    if(connected.size() < limitNbVarCache) return true;
    if(connected.size() < limitNbVarCacheDynamic) return true;
    return false;
  } // cacheIsActivated


  /**
     Update the dynamic limit.
   */
  void updateDynamicLimit()
  {
    limitNbVarCacheDynamic = 0;
    if(m_staticLimit) return;
#if 0
    for(unsigned i = 0 ; i<nbPosHitCacheVarSize.size() ; i++)
    {
      if(!nbTestCacheVarSize[i]) continue;
      if(nbPosHitCacheVarSize[i]) std::cout << "\033[1m\033[31m";
      std::cout << i << "("<< nbTestCacheVarSize[i] << "/" << nbPosHitCacheVarSize[i] << ") ";
      if(nbPosHitCacheVarSize[i]) std::cout << "\033[0m";
    }
    std::cout << "\n";
#endif
    
    for(unsigned i = 0 ; i<nbPosHitCacheVarSize.size() ; i++)
    {
      if(nbPosHitCacheVarSize[i]) limitNbVarCacheDynamic = i;
      nbPosHitCacheVarSize[i] >>= 1;      
    }
    
    limitNbVarCacheDynamic *= ratioDynamicLimit;
    m_out << "c Update dynamic limit: " << limitNbVarCacheDynamic
          << "/" << limitNbVarCache << "\n";
  } // updateDynamicLimit
  
  
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
    // if(nbCallCall > 114000) {exit(0);}
    
    if(!solver->solve(setOfVar)) return 0;    
    solver->whichAreUnits(setOfVar, unitsLit); // collect unit literals
    specs->preUpdate(unitsLit);

    T ret = 1, curr;
    
    // compute the connected composant
    std::vector<Var> reallyPresent;
    std::vector< std::vector<Var> > varConnected;
    
    int nbComponent = specs->computeConnectedComponent(
        varConnected, setOfVar, freeVariable, reallyPresent);

    if(++m_limitUpdateCounter > m_limitUpdate)
    {
      updateDynamicLimit();
      m_limitUpdateCounter = 0;
    }
    
    // consider each connected component.
    if(nbComponent)
    {
      nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for(int cp = 0 ; cp<nbComponent ; cp++)
      {
        std::vector<Var> &connected = varConnected[cp];
        bool cacheActivated = cacheIsActivated(connected);
        
        TmpEntry<T> cb = cacheActivated ? m_cache->searchInCache(connected)
                         : NULL_CACHE_ENTRY;
        
        if(cacheActivated) nbTestCacheVarSize[connected.size()]++;
        if(cacheActivated && cb.defined)
        {
          nbPosHitCacheVarSize[connected.size()]++;
          ret *= cb.getValue();
        }
        else
        {
          // recursive call
          std::vector<Var> currPriority;
          computePrioritySubSet(connected, priorityVar, currPriority);
          ret *= (curr = computeDecisionNode(connected, currPriority, out));

          if(cacheActivated) m_cache->addInCache(cb, curr);
        }
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
        m_hCutSet->computeCutSet(connected, priorityVar);        
        callPartitioner++;
      }

    // search the next variable to branch on
    std::vector<Var> &inVars = (priorityVar.size()) ? priorityVar : connected;
    Var v = m_hVar->selectVariable(inVars, *specs);
    if(v == var_Undef) return 1;
    
    Lit l = Lit::makeLit(v, m_hPhase->selectPhase(v));
    nbDecisionNode++;    
    
    // compile the formula where l is assigned to true
    std::vector<Lit> unitLitPos, unitLitNeg;
    std::vector<Var> freeVarPos, freeVarNeg;    
    
    solver->pushAssumption(l);
    T pos = computeNbModel_(connected, unitLitPos, freeVarPos, priorityVar, out);
    pos *= problem->computeWeightUnitFree<T>(unitLitPos, freeVarPos);
    solver->popAssumption();

    solver->pushAssumption(~l);
    T neg = computeNbModel_(connected, unitLitNeg, freeVarNeg, priorityVar, out);
    neg *= problem->computeWeightUnitFree<T>(unitLitNeg, freeVarNeg);
    solver->popAssumption();

    return pos + neg;
  }// computeDecisionNode

  
  /**
     Compute the number of model using the trace of a SAT solver.

     \return the number of models.
  */
  T computeNbModel(std::ostream &out)
  {
    std::vector<Var> freeVariable, setOfVar, priorityVar;
    std::vector<Lit> unitsLit;

    for(int i = 1 ; i <= specs->getNbVariable() ; i++) setOfVar.push_back(i);

    if(problem->isUnsat() || !solver->warmStart(29, 11, setOfVar, m_out)) return T(0);    
    T d = computeNbModel_(setOfVar, unitsLit, freeVariable, priorityVar, out);    
    return d * problem->computeWeightUnitFree<T>(unitsLit, freeVariable);
  }// computeNbModel

 public:
  
  /**
     The method called to run the model counter.
   */
  void run(po::variables_map &vm)
  {
    T nbModels = computeNbModel(m_out);
    printFinalStats(m_out);
    m_out << "s " << std::fixed << nbModels << "\n";
  } // run
};
} // d4
