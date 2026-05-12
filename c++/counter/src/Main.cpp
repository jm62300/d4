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
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "CounterDemo.hpp"
#include "ParserDimacs.hpp"
#include "src/binding/json/Binding.hpp"
#include "src/binding/json/SchemaProviders.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"

namespace fs = std::filesystem;
d4::MethodManager* methodRun = nullptr;

/**
   The main function.

   Usage: counter -i INPUT [-h]

   -i INPUT   Path to the input DIMACS file (required).

   Examples:
     counter -i formula.cnf
     counter -i formula.cnf --solver.solverName 1
*/
int main(int argc, char** argv) {
  auto start = std::chrono::system_clock::now();

  std::string inputPath;
  bool showHelp = false;

  // Simple manual scan for launcher options
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      showHelp = true;
    } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
      inputPath = argv[++i];
    }
  }

  if (showHelp || inputPath.empty()) {
    if (!showHelp && inputPath.empty())
      std::cerr << "Missing required argument: -i INPUT\n";

    std::cout << "USAGE: " << argv[0] << " -i INPUT [Overrides...]\n"
              << "  -i, --input   Path to the input DIMACS file (required)\n"
              << "  -h, --help    Show this help screen\n"
              << "\n\033[1mConfiguration Help:\033[0m\n"
              << "The configuration follows this schema. Use --key=value to "
                 "override fields.\n";
    d4::to_pretty_tree(d4::generate_schema<d4::ConfigurationDpllStyleMethod>(),
                       std::cout);
    std::cout << std::endl;
    return showHelp ? 0 : 1;
  }

  // Check if input file exists
  if (!fs::exists(inputPath)) {
    std::cerr << "ERROR! Input file does not exist: " << inputPath << "\n";
    return 1;
  }

  parser::Formula formula;
  parser::ParserDimacs parserDimacs;
  parserDimacs.parse_DIMACS(inputPath, formula);

#if 0
  std::cout << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input "
               "formula\033[0m\n";
  initProblem->displayStat(std::cout, "c [INITIAL INPUT] ");
  std::cout << "c\n";

  if (vm["translate"].as<std::string>() != "none") {
    std::cout << "c [TRANSLATION] Translate the input formula: "
              << vm["input-type"].as<std::string>() << " -> "
              << vm["translate"].as<std::string>() << '\n';
    d4::ProblemManager* tmp =
        initProblem->translate(d4::ProblemTranslateTypeManager::getInputType(
            vm["translate"].as<std::string>()));
    delete initProblem;
    initProblem = tmp;
  }

  // run the method asked.
  d4::MethodName methodName = d4::MethodNameManager::getMethodName("counting");
#endif

// preproc.
#if 0
  d4::ConfigurationPeproc configPreproc = parsePreprocConfiguration(vm);
  configPreproc.inputType = initProblem->getProblemType();
  ProblemManager* problem =
      d4::MethodManager::runPreproc(configPreproc, initProblem, std::cout);
#endif

  // Build the final configuration using command line arguments.
  auto config = d4::from_json_string_and_argv<d4::ConfigurationDpllStyleMethod>(
      "{}", argc, argv);

  // count.
  counterDemo(config, formula);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "c [COUNTER] Elapsed time: " << elapsed.count() << " seconds\n";

  return EXIT_SUCCESS;
}  // main