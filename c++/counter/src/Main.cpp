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

#include "Counter.hpp"
#include "OptionCounter.hpp"
#include "ParserCircuit.hpp"
#include "ParserDimacs.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/preproc/PreprocManager.hpp"

namespace fs = std::filesystem;
d4::MethodManager* methodRun = nullptr;

// Run BiPe preprocessing on the formula in place.
static void runPreproc(parser::Formula& formula,
                       const bipe::OptionPreproc& optionPreproc) {
  bipe::PreprocManager preprocManager;

  // quantifications[0] holds the projection scope when doing projected
  // counting; fall back to all variables for standard model counting.
  std::vector<int> projected;
  if (formula.quantifications[0].size())
    projected = formula.quantifications[0];
  else
    for (unsigned i = 1; i <= formula.nbVar; i++) projected.push_back(i);

  // Variables whose positive and negative literal weights differ must not be
  // eliminated: removing them would change the weighted count.
  std::vector<int> varProtected;
  for (int i = 1; i <= formula.nbVar; i++) {
    std::string w1 = formula.weightMap.find(i) != formula.weightMap.end()
                         ? formula.weightMap[i]
                         : "";
    std::string w2 = formula.weightMap.find(-i) != formula.weightMap.end()
                         ? formula.weightMap[-i]
                         : "";
    if (w1 != w2) varProtected.push_back(i);
  }

  preprocManager.run(formula.nbVar, formula.clauses, projected, varProtected,
                     optionPreproc);

  // Reflect the (possibly narrowed) projected set back into the formula so the
  // counter performs projected model counting whenever preprocessing shrank the
  // scope below all variables (e.g. --preproc.project-on-input narrows it to the
  // bipartition input set). When the scope still covers every variable this
  // clears the quantification so standard model counting is used.
  if (formula.quantifications.empty()) formula.quantifications.emplace_back();
  if (projected.size() < formula.nbVar)
    formula.quantifications[0] = projected;
  else
    formula.quantifications[0].clear();
}  // runPreproc

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

  // 1. Initialize configuration and registry
  d4::OptionDpllStyleMethod options;
  d4::OptionCounter optionCounter;
  bipe::OptionPreproc optionPreproc;

  d4::OptionRegistry registry;
  options.registerTo(registry);
  optionCounter.registerTo(registry);
  optionPreproc.registerTo(registry);

  registry.parseArgv(argc, argv);

  if (showHelp || inputPath.empty()) {
    if (!showHelp && inputPath.empty())
      std::cerr << "Missing required argument: -i INPUT\n";

    std::cout << "USAGE: " << argv[0] << " -i INPUT [Overrides...]\n"
              << "  -i, --input   Path to the input DIMACS file (required)\n"
              << "  -h, --help    Show this help screen\n";

    registry.displayHelp(std::cout);

    return showHelp ? 0 : 1;
  }

  // Check if input file exists
  if (!fs::exists(inputPath)) {
    std::cerr << "ERROR! Input file does not exist: " << inputPath << "\n";
    return 1;
  }

  parser::Formula formula;
  const std::string informat = optionCounter.informat.get();

  if (informat == "circuit") {
    parser::ParserCircuit parserCircuit;
    parserCircuit.parse_circuit(inputPath, formula);
  } else {
    parser::ParserDimacs parserDimacs;
    parserDimacs.parse_DIMACS(inputPath, formula);
    runPreproc(formula, optionPreproc);
  }

  counter(options, optionCounter, formula);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "c [COUNTER] Elapsed time: " << elapsed.count() << " seconds\n";

  return EXIT_SUCCESS;
}  // main