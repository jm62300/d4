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

#include <iostream>
#include <cassert>
#include <vector>
#include <boost/program_options.hpp>

#include <src/problem/ProblemManager.hpp>
#include <src/preprocs/PreprocManager.hpp>
#include <src/specs/SpecManager.hpp>
#include <src/heuristics/ScoringMethod.hpp>
#include <src/heuristics/PhaseHeuristic.hpp>
#include <src/heuristics/PartitioningHeuristic.hpp>
#include <src/solvers/WrapperSolver.hpp>
#include <src/utils/EquivExtractor.hpp>

/**
   The main function!
*/
int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::options_description desc{"Options"};
  desc.add_options()
#include "option.dsc"
      ;

  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);

  try
  {    
    po::notify(vm);
  }
  catch (const po::error &ex)
  {
    std::cerr << ex.what() << '\n';
    exit(1);
  }

  // help or problem with the command line
  if (vm.count("help") || !vm.count("input"))
  {
    if(!vm.count("help"))
      std::cout << "Some parameters are missing, please read the README\n";
    std::cout << "USAGE: " << argv[0] << " -i INPUT -m METH [OPTIONS]\n";
    std::cout << desc << '\n';
    exit(!vm.count("help"));
  }
  
  d4::ProblemManager *problem = d4::ProblemManager::makeProblemManager(vm);
  problem->display(std::cout);

  d4::PreprocManager *preproc = d4::PreprocManager::makePreprocManager(vm);  
  d4::ProblemManager *preprocProblem = preproc->run(*problem);
  preprocProblem->display(std::cout);

  d4::SpecManager *specManager = d4::SpecManager::makeSpecManager(vm, *problem);
  specManager->showFormula(std::cout);

  std::vector<d4::Lit> l;
  l.push_back(d4::Lit(1,false));
  specManager->preUpdate(l);
  specManager->showCurrentFormula(std::cout);
  specManager->postUpdate(l);
  specManager->showCurrentFormula(std::cout);


  d4::WrapperSolver *solver = d4::WrapperSolver::makeWrapperSolver(vm);
  solver->initSolver(*problem);
  
  d4::ScoringMethod *sm = d4::ScoringMethod::makeScoringMethod(vm, *specManager, *solver);
  assert(sm);
  std::cout << sm->computeScore(2) << "\n";

  d4::PhaseHeuristic *phase = d4::PhaseHeuristic::makePhaseHeuristic(vm, *specManager, *solver);
  assert(phase);
  std::cout << phase->selectPhase(2) << "\n";

  d4::PartitioningHeuristic *partition = d4::PartitioningHeuristic::makePartitioningHeuristic(vm, *specManager, *solver);
  assert(partition);
  
  delete partition;
  delete phase;
  delete sm;
  delete problem;
  delete preprocProblem;
  delete preproc;
  delete specManager;
  delete solver;

  return 0;
}// main
