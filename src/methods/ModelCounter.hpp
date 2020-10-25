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
  unsigned limitCacheDyn;
  unsigned nbDecisionNode;
  unsigned freqLimitDyn;
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

    showRun(std::cout);
    showInter(std::cout);
    printFinalStats(std::cout);
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
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << limitCacheDyn
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
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "Mem(MB)" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#equivCall" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#Dec. Node" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "#partioner" 
        << "|" << std::setw(WIDTH_PRINT_COLUMN_MC) << "limitDyn" 
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

  
#if 0  
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
    showRun(); nbCallCall++;
    s.rebuildWithConnectedComponent(setOfVar);

    if(!s.solveWithAssumptions()) return 0;
    s.collectUnit(setOfVar, unitsLit); // collect unit literals

    occManager->preUpdate(unitsLit);

    // compute the connected composant
    vec<Var> reallyPresent;
    vec< vec<Var> > varConnected;
    int nbComponent = occManager->computeConnectedComponent(varConnected, setOfVar, freeVariable, reallyPresent);

    T ret = 1, curr;
    if(nbComponent)
    {
      nbSplit += (nbComponent > 1) ? nbComponent : 0;
      for(int cp = 0 ; cp<nbComponent ; cp++)
      {
        vec<Var> &connected = varConnected[cp];
        bool localCache = optCached == 1 || (optCached > 1 && !cache->shouldNotCache(connected.size(), bm));

        occManager->updateCurrentClauseSet(connected);
        TmpEntry<T> cb = localCache ? cache->searchInCache(connected, bm) : NULL_CACHE_ENTRY;

        if(localCache && cb.defined) ret *= cb.getValue();
        else
        {
          // recursive call
          vec<Var> currPriority;
          computePrioritySubSet(connected, priorityVar, currPriority);
          ret *= (curr = computeDecisionNode(connected, currPriority));

          if(localCache) cache->addInCache(cb, curr);
        }
        occManager->popPreviousClauseSet();
      }
    }// else we have a tautology

    occManager->postUpdate(unitsLit);
    return ret;
  }// computeNbModel_



  /**
     This function select a variable and compile a decision node.

     @param[in] connected, the set of variable present in the current problem
     \return the compiled formula
  */
  T computeDecisionNode(vec<Var> &connected, vec<Var> &priorityVar)
  {
    if(pv && !priorityVar.size() && connected.size() > 10 && connected.size() < 5000)
      {
        vec<int> cutSet;
        pv->computePartition(connected, cutSet, priorityVar, vs->getScoringFunction());
        callPartitioner++;
      }

    Var v = var_Undef;
    if(priorityVar.size()) v = vs->selectVariable(priorityVar); else v = vs->selectVariable(connected);
    if(v == var_Undef) return 1;

    Lit l = mkLit(v, optReversePolarity - vs->selectPhase(v));
    nbDecisionNode++;

    // compile the formula where l is assigned to true
    vec<Lit> unitLitPos, unitLitNeg;
    vec<Var> freeVarPos, freeVarNeg;

    (s.assumptions).push(l);
    T pos = computeNbModel_(connected, unitLitPos, freeVarPos, priorityVar);
    pos *= computeWeightUnitFree(unitLitPos, freeVarPos);
    (s.assumptions).pop();
    (s.cancelUntil)((s.assumptions).size());

    (s.assumptions).push(~l);
    T neg = computeNbModel_(connected, unitLitNeg, freeVarNeg, priorityVar);
    neg *= computeWeightUnitFree(unitLitNeg, freeVarNeg);
    (s.assumptions).pop();
    (s.cancelUntil)((s.assumptions).size());

    return neg + pos;
  }// computeDecisionNode



  inline void init(int nbClauses, int maxSizeClause, vec<double> &wl, OptionManager &optList,
                   vec<bool> &isProjectedVar)
  {
    wl.copyTo(weightLit);
    for(int i = 0 ; i<wl.size()>>1 ; i++) s.newVar();
    for(int i = 0 ; i<s.nVars() ; i++)
    {
      weightVar.push(weightLit[i<<1] + weightLit[(i<<1) | 1]);
    }
    limitCacheDyn = s.nVars();

    callPartitioner = callEquiv = 0;
    optCached = optList.optCache;
    optReversePolarity = optList.reversePolarity;

    optList.printOptions();

    // initialized the data structure
    prepareVecClauses(clauses, s);
    occManager = new DynamicOccurrenceManager(0, s.nVars(), 0);

    cache = new CacheCNF<T>(optList.reduceCache, optList.strategyRedCache);
    cache->initHashTable(occManager->getNbVariable(), nbClauses, maxSizeClause);

    vs = new VariableHeuristicInterface(s, occManager, optList.varHeuristic,
                                        optList.phaseHeuristic, isProjectedVar);

    if(!strcmp(optList.cacheRepresentation, "CL"))
      bm = new BucketManager<T>(occManager, nbClauses, s.nVars(), maxSizeClause, optList.strategyRedCache);
    else if(!strcmp(optList.cacheRepresentation, "HC"))
      bm = new BucketManagerHC<T>(occManager, nbClauses, s.nVars(), maxSizeClause, optList.strategyRedCache);
    else bm = new BucketManagerSym<T>(occManager, nbClauses, s.nVars(), maxSizeClause, optList.strategyRedCache);

    pv = PartitionerInterface::getPartitioner(s, occManager, optList);
    bm->setFixeFormula(optList.cacheStore);

    // statistics initialization
    nbSplit = nbCallCall = 0;
    currentTime = cpuTime();
    nbDecisionNode = nbNodeInCall = 0;

    stampIdx = 0;
    stampVar.initialize(s.nVars(), 0);
    em.initEquivManager(s.nVars());
  }// init

public:

  T getWeightVar(Var v){return T(weightVar[v]);}

  /**
     Compute the value for free and unit variables.

     @param[in] units, the units variables
     @param[in] frees, the free variables

     \return the right value
   */
  inline T computeWeightUnitFree(vec<Lit> &units, vec<Var> &frees)
  {
    T tmp = 1;
    for(int i = 0 ; i<units.size() ; i++)
      if(vs->isProjected(var(units[i]))) tmp *= T(weightLit[toInt(units[i])]);
    for(int i = 0 ; i<frees.size() ; i++)
      if(vs->isProjected(frees[i])) tmp *= T(weightVar[frees[i]]);
    return tmp;
  } // computeValue




  /**
     Constructor of model counter that does not take a formula as
     input (the formula is given later).

     @param[in] fWeights, the vector of literal's weight
     @param[in] optList, the options
     @param[in] isProjectedVar, boolean vector used to decide if a variable is projected (true) or not (false)
  */
  ModelCounter(int nbClauses, int maxSizeClause, vec<double> &wl, OptionManager &optList,
               vec<bool> &isProjectedVar)
  {
    init(nbClauses, maxSizeClause, wl, optList, isProjectedVar);
  } // ModelCounter


  /**
     Constructor of model counter.

     @param[in] cnf, set of clauses
     @param[in] fWeights, the vector of literal's weight
     @param[in] optList, the options
     @param[in] isProjectedVar, boolean vector used to decide if a variable is projected (true) or not (false)
  */
  ModelCounter(vec<vec<Lit> > &cnf, vec<double> &wl, OptionManager &optList, vec<bool> &isProjectedVar)
  {
    // init the model counter's date structures
    int maxSizeClause = 0;
    for(int i = 0 ; i<cnf.size() ; i++) if(cnf[i].size() > maxSizeClause) maxSizeClause = cnf[i].size();
    init(cnf.size(), maxSizeClause, wl, optList, isProjectedVar);

    // init the solver
    for(int i = 0 ; i<cnf.size() ; i++) s.addClause_(cnf[i]);

    // test the satifiability of the input formula
    if(!s.solveWithAssumptions()){printf("c The formula is unsatisfiable\ns 0\n"); exit(0);}
    s.simplify();
    s.remove_satisfied = false;
    s.setNeedModel(false);

    // add the clauses to the occurrence manager
    vec<vec<Lit> > reduceCnf;
    for(int i = 0 ; i<cnf.size() ; i++)
    {
      bool isSAT = false;
      reduceCnf.push();
      vec<Lit> &cl = cnf[i], &redCl = reduceCnf.last();

      for(int j = 0 ; !isSAT && j<cl.size() ; j++)
      {
        if(s.value(cl[j]) == l_Undef) redCl.push(cl[j]);
        isSAT = isSAT || s.value(cl[j]) == l_True;
      }


      if(isSAT) reduceCnf.pop();
      else assert(redCl.size());
    }

    freqLimitDyn = optList.freqLimitDyn;
    occManager->initFormula(reduceCnf);
    cache->setInfoFormula(s.nVars(), reduceCnf.size(), occManager->getMaxSizeClause());
  }// ModelCounter

  ~ModelCounter()
  {
    if(pv) delete pv;
    delete cache; delete vs; delete bm;
    delete occManager;
  }

  /**
     Initialize the assumption in order to compute the number of model
     under this one.

     @param[in] assums, the assumption
   */
  void initAssumption(vec<Lit> &assums)
  {
    s.cancelUntil(0);
    assums.copyTo(s.assumptions);
  }// initAssumption


  /**
     Compute the number of model using the trace of a SAT solver.

     \return the number of models
  */
  T computeNbModel(bool verb = true)
  {
    vec<Var> freeVariable, setOfVar, priorityVar;
    vec<Lit> unitsLit;

    for(int i = 0 ; i<s.nVars() ; i++) setOfVar.push(i);
    T d = computeNbModel_(setOfVar, unitsLit, freeVariable, priorityVar);

    if(verb) printFinalStatsCache();

    T computeWeight = 1;
    for(int i = 0 ; i<freeVariable.size() ; i++)
      if(vs->isProjected(freeVariable[i])) computeWeight *= T(weightVar[freeVariable[i]]);
    for(int i = 0 ; i<unitsLit.size() ; i++)
      if(vs->isProjected(var(unitsLit[i]))) computeWeight *= T(weightLit[toInt(unitsLit[i])]);

    return d * computeWeight;
  }// computeNbModel


  /**
     Compute the number of model using the trace of a SAT solver.

     \return the number of models
  */
  T computeNbModel(vec<Var> &setOfVar, bool verb = true)
  {
    vec<Var> freeVariable, priorityVar;
    vec<Lit> unitsLit;

    // we need to collect the variabels they are in the assumption and not in setOfVar
    vec<Lit> assumsLit;
    vec<bool> markedVar;

    for(int i = 0 ; i<s.nVars() ; i++) markedVar.push(false);
    for(int i = 0 ; i<setOfVar.size() ; i++) markedVar[setOfVar[i]] = true;
    for(int i = 0 ; i<s.assumptions.size() ; i++)
      if(!markedVar[var(s.assumptions[i])]) assumsLit.push(s.assumptions[i]);

    occManager->preUpdate(assumsLit);
    T d = computeNbModel_(setOfVar, unitsLit, freeVariable, priorityVar);
    occManager->postUpdate(assumsLit);

    if(verb) printFinalStatsCache();

    T computeWeight = 1;
    for(int i = 0 ; i<freeVariable.size() ; i++)
      if(vs->isProjected(freeVariable[i])) computeWeight *= T(weightVar[freeVariable[i]]);
    for(int i = 0 ; i<unitsLit.size() ; i++)
      if(vs->isProjected(var(unitsLit[i]))) computeWeight *= T(weightLit[toInt(unitsLit[i])]);

    return d * computeWeight;
  }// computeNbModel
#endif

  
  /**
     The method called to run the model counter.
   */
  void run()
  {
    
  } // run
  
};
} // d4

#endif

