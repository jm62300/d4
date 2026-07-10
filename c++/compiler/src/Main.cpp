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

#include "CompilerDemo.hpp"
#include "OptionCompiler.hpp"
#include "ParserDimacs.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/preproc/PreprocManager.hpp"

namespace fs = std::filesystem;

/**
   The main function.

   Usage: compilation -i INPUT [-h]

   -i INPUT   Path to the input DIMACS file (required).

   Examples:
     counter -i formula.cnf
     counter -i formula.cnf --solver.solverName glucose
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
  d4::OptionCompiler optionCompiler;
  bipe::OptionPreproc optionPreproc;

  d4::OptionRegistry registry;
  options.registerTo(registry);
  optionCompiler.registerTo(registry);
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
  parser::ParserDimacs parserDimacs;
  parserDimacs.parse_DIMACS(inputPath, formula);

  // we cannot apply stronger preprocessing there.
  optionPreproc.optionPreprocMethod = bipe::PreprocMethod::EQUIV_FULL;

  // preproc.
  bipe::PreprocManager preprocManager;
  std::vector<int> projected;
  if (formula.quantifications[0].size())
    projected = formula.quantifications[0];
  else
    for (unsigned i = 1; i <= formula.nbVar; i++) projected.push_back(i);

  preprocManager.run(formula.nbVar, formula.clauses, projected,
                     std::vector<int>(), optionPreproc);

  if (optionCompiler.implicant.get()) {
    std::cout << "c Transform the formula to compute implicant\n";

    // split the formula.
    // rename negative literals.
    for (auto& cl : formula.clauses) {
      for (auto& l : cl)
        if (l < 0) l = std::abs(l) + formula.nbVar;
    }

    // add the binary clauses.
    for (int i = 1; i <= formula.nbVar; i++)
      formula.clauses.push_back({-i, -(i + (int)formula.nbVar)});

    // increase the number of variables.
    formula.nbVar *= 2;
  }

  // compiler.
  compiler(options, optionCompiler, formula);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "c [COUNTER] Elapsed time: " << elapsed.count() << " seconds\n";

  return EXIT_SUCCESS;
}  // main
