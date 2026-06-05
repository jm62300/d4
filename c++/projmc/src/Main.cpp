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
#include <iostream>
#include <string>

#include "OptionProjMcCounter.hpp"
#include "ParserDimacs.hpp"
#include "ProjMcCounter.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionProjMcMethod.hpp"
#include "src/preproc/PreprocManager.hpp"

namespace fs = std::filesystem;
d4::MethodManager* methodRun = nullptr;

// Run BiPe preprocessing on the formula in place.
// For projected model counting the projection scope is used directly
// so BiPe knows which variables must be preserved.
static void runPreproc(parser::Formula& formula,
                       const bipe::OptionPreproc& optionPreproc) {
  bipe::PreprocManager preprocManager;

  // quantifications[0] is the projection scope; it must always be non-empty
  // here (checked in main), so we use it directly as the projection set.
  std::vector<int> projected = formula.quantifications[0];

  // Variables with asymmetric weights must not be eliminated.
  std::vector<int> varProtected;
  for (int i = 1; i <= (int)formula.nbVar; i++) {
    std::string w1 = formula.weightMap.count(i) ? formula.weightMap[i] : "";
    std::string w2 = formula.weightMap.count(-i) ? formula.weightMap[-i] : "";
    if (w1 != w2) varProtected.push_back(i);
  }

  preprocManager.run(formula.nbVar, formula.clauses, projected, varProtected,
                     optionPreproc);
}  // runPreproc

/**
   The main function.

   Usage: projmc -i INPUT [-h]

   -i INPUT   Path to the projected DIMACS file (required; must declare a
              projection scope via 'p show' or 'c p show' header lines).

   Examples:
     projmc -i formula.cnf
     projmc -i formula.cnf --refinement false
     projmc -i formula.cnf --preproc.vivification false
*/
int main(int argc, char** argv) {
  auto start = std::chrono::system_clock::now();

  std::string inputPath;
  bool showHelp = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      showHelp = true;
    } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
      inputPath = argv[++i];
    }
  }

  d4::OptionProjMcMethod options;
  d4::OptionProjMcCounter optionCounter;
  bipe::OptionPreproc optionPreproc;

  d4::OptionRegistry registry;
  options.registerTo(registry);
  optionCounter.registerTo(registry);
  optionPreproc.registerTo(registry);

  registry.parseArgv(argc, argv);

  if (showHelp || inputPath.empty()) {
    if (!showHelp && inputPath.empty())
      std::cerr << "Missing required argument: -i INPUT\n";

    std::cout
        << "USAGE: " << argv[0] << " -i INPUT [Overrides...]\n"
        << "  -i, --input   Path to the projected DIMACS file (required)\n"
        << "  -h, --help    Show this help screen\n";

    registry.displayHelp(std::cout);
    return showHelp ? 0 : 1;
  }

  if (!fs::exists(inputPath)) {
    std::cerr << "ERROR! Input file does not exist: " << inputPath << "\n";
    return 1;
  }

  parser::Formula formula;
  parser::ParserDimacs parserDimacs;
  parserDimacs.parse_DIMACS(inputPath, formula);

  if (formula.quantifications.empty() || formula.quantifications[0].empty()) {
    std::cerr << "ERROR! No projection scope found in the input file.\n"
              << "       Use 'p show <vars>' or 'c p show <vars>' to declare "
                 "projected variables.\n";
    return 1;
  }

  runPreproc(formula, optionPreproc);

  std::cout << formula << '\n';
  projMcCounter(options, optionCounter, formula);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "c [PROJMC] Elapsed time: " << elapsed.count() << " seconds\n";

  return EXIT_SUCCESS;
}  // main
