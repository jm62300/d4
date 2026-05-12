/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <signal.h>
#include <unistd.h>

#include <boost/program_options.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "BManager.hpp"
#include "ParserDimacs.hpp"
#include "src/bipartition/methods/Method.hpp"
#include "src/bipartition/option/Option.hpp"
#include "src/eliminator/Eliminator.hpp"
#include "src/preproc/PreprocManager.hpp"
#include "src/reducer/Method.hpp"
#include "src/utils/Problem.hpp"
#include "src/utils/Timer.hpp"

bipe::bipartition::Method* methodRun = nullptr;
namespace po = boost::program_options;

/**
 * @brief Compute a bipartition.
 *
 * @param option, is a map to get the options.
 */
void computeB(po::variables_map& vm, bipe::Problem& pb,
              const bipe::Timer& timer) {
  BManager b;
  std::vector<bipe::Gate> gates;
  std::vector<bipe::Var> input, output;
  bool res = b.run(vm, pb, input, output, gates, timer);

  if (res) {
    std::cout << "v ";
    std::sort(input.begin(), input.end());
    for (auto v : input) std::cout << v << " ";
    std::cout << "0\n";
  } else {
    std::cout << "s UNSATISFIABLE\n";
    std::cout << "v 0\n";
  }
}  // computeB

/**
 * @brief Compute a bipartition.
 *
 * @param vm is a map to get the options.
 * @param pb is the problem we want to process.
 */
void computeBE(po::variables_map& vm, bipe::Problem& pb,
               const bipe::Timer& timer) {
  BManager b;
  std::vector<bipe::Gate> gates;
  std::vector<bipe::Var> input, output;
  bool res = b.run(vm, pb, input, output, gates, timer);

  bipe::eliminator::Eliminator e;
  std::vector<bipe::Lit> eliminated;

  std::vector<std::vector<bipe::Lit>>& cnf = pb.getClauses();
  e.eliminate(pb.getNbVar(), cnf, input, gates, eliminated,
              vm["elimination-verbosity"].as<bool>(), cnf.size(), timer);

  // print out the formula.
  std::cout << "p cnf " << pb.getNbVar() << ' '
            << cnf.size() + eliminated.size() << '\n';
  for (auto& cl : cnf) {
    for (auto& l : cl) std::cout << l.human() << ' ';
    std::cout << "0\n";
  }

  std::cout << "c Unit clauses to block additional models.\n";
  for (auto& l : eliminated) std::cout << l.human() << " 0\n";
}  // computeB

/**
 * @brief Call the bipartition algorithm and then try to eliminate output
 * variables by considering resolution or gate definitions.
 *
 * @param vm is the options.
 * @param pb is the problem we are dealing with (CNF formula).
 */
void computeBER(po::variables_map& vm, bipe::Problem& pb,
                const bipe::Timer& timer) {
  BManager b;
  std::vector<bipe::Gate> gates;
  std::vector<bipe::Var> input, output;
  bool res = b.run(vm, pb, input, output, gates, timer);

  bipe::eliminator::Eliminator e;
  bipe::reducer::Method* r =
      bipe::reducer::Method::makeMethod("combinaison", std::cout);
  std::vector<bipe::Lit> eliminated;

  std::vector<std::vector<bipe::Lit>> cnf = pb.getClauses();
  unsigned initSize = cnf.size();

  r->run(pb.getNbVar(), cnf, 5, true, cnf, timer);

  e.eliminate(pb.getNbVar(), cnf, input, gates, eliminated,
              vm["elimination-verbosity"].as<bool>(), initSize, timer);

  r->run(pb.getNbVar(), cnf, 5, true, cnf, timer);

  // print out the formula.
  std::cout << "p cnf " << pb.getNbVar() << ' '
            << cnf.size() + eliminated.size() << '\n';
  for (auto& cl : cnf) {
    for (auto& l : cl) std::cout << l.human() << ' ';
    std::cout << "0\n";
  }

  std::cout << "c Unit clauses to block additional models.\n";
  for (auto& l : eliminated) std::cout << l.human() << " 0\n";
}  // computeBE

int main(int argc, char** argv) {
  std::cout << "c [BIPE] Version 2.0\n";
  po::options_description desc{"Options"};
#include "option.dsc"

  po::variables_map vm;
  try {
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const po::error& ex) {
    std::cerr << ex.what() << '\n';
    exit(1);
  }
  std::cout << "c [BIPE] Parsing options done\n";

  // help or problem with the command line
  if (vm.count("help") || !vm.count("input")) {
    if (!vm.count("help"))
      std::cout << "Some parameters are missing, please read the README\n";
    std::cout << "USAGE: " << argv[0] << " -i CNF_INPUT -m METHOD\n";
    std::cout << desc << '\n';
    exit(!vm.count("help"));
  }

  // parse the problem.
  std::cout << "c [BIPE] Input: " << vm["input"].as<std::string>() << "\n";
  ParserDimacs parser;
  ParsedProblem parsedProblem;
  parser.parse_DIMACS(vm["input"].as<std::string>(), parsedProblem);

  bipe::OptionPreproc optionPreproc;
  bipe::PreprocMethod preprocMethod = bipe::NONE;

  std::cout << "c [BIPE] Parsing input done\n";

  // set the alarm if needed.
  if (vm["timeout"].as<unsigned>() != 0) {
    std::cout << "c [BIPE] A timeout is set to: "
              << vm["timeout"].as<unsigned>() << "\n";
    optionPreproc.timeout = vm["timeout"].as<unsigned>();
  } else
    std::cout << "c [BIPE] No timeout\n";

  if (vm["method"].as<std::string>() == "equiv-light") {
    preprocMethod = bipe::EQUIV_LIGHT;
  }

  bipe::PreprocManager preprocManager;
  preprocManager.run(parsedProblem.nbVar, parsedProblem.clauses,
                     parsedProblem.projected, std::vector<int>(), preprocMethod,
                     optionPreproc);
}
