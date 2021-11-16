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
#include "../../src/methods/Counter.hpp"
#include <boost/program_options.hpp>
#include <iostream>

/**
   The main function!
*/
int main(int argc, char **argv) {
  namespace po = boost::program_options;
  namespace mpz = boost::multiprecision;

  po::options_description desc{"Options"};
  desc.add_options()
#include "option.dsc"
      ;

  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);

  try {
    po::notify(vm);
  } catch (const po::error &ex) {
    std::cerr << ex.what() << '\n';
    exit(1);
  }

  // help or problem with the command line
  if (vm.count("help")) {
    std::cout << desc << '\n';
    exit(!vm.count("help"));
  }

  unsigned nbVar = 3;
  d4::ProblemManagerCnf problem;
  problem.setNbVar(nbVar);

  std::vector<double> &weightLit = problem.getWeightLit();
  std::vector<double> &weightVar = problem.getWeightVar();
  weightLit.resize((nbVar + 1) << 1, 1);
  weightVar.resize(nbVar + 1, 2);

  std::vector<std::vector<d4::Lit>> &clauses = problem.getClauses();
  clauses.push_back({d4::Lit::makeLitFalse(1), d4::Lit::makeLitTrue(2)});
  clauses.push_back({d4::Lit::makeLitTrue(1), d4::Lit::makeLitTrue(3)});

  problem.display(std::cout);

  d4::LastBreathPreproc lastBreath;
  d4::PreprocManager *preproc =
      d4::PreprocManager::makePreprocManager(vm, std::cout);
  d4::ProblemManager *preprocProblem = preproc->run(problem, lastBreath);

  boost::multiprecision::mpf_float::default_precision(50);
  d4::Counter<mpz::mpf_float> *method =
      d4::Counter<mpz::mpf_float>::makeCounter(vm, preprocProblem, "counting",
                                               true, 50, std::cout, lastBreath);

  std::vector<d4::Var> setOfVar;
  for (unsigned i = 1; i <= nbVar; i++)
    setOfVar.push_back(i);

  mpz::mpf_float v = method->count(setOfVar, std::cout);
  std::cout << "s " << v << "\n";
  delete method;
  return 0;
} // main
