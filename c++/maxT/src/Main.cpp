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
#include <signal.h>

#include <chrono>
#include <filesystem>
#include <iostream>

#include "MaxTSolver.hpp"
#include "ParserDimacs.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionMaxTMethod.hpp"

namespace fs = std::filesystem;

using namespace d4;
MethodManager* methodRun = nullptr;

/**
 * @brief Catch the signal that asks for stopping the method which is
 * running.
 *
 * @param signum is the signal.
 */
static void signalHandler(int signum) {
  std::cout << "c [MAIN] Method stop\n";
  if (methodRun != nullptr) methodRun->interrupt();
  exit(signum);
}  // signalHandler

/**
   The main function.

   Usage: maxT -i INPUT [-h]

   -i INPUT   Path to the input DIMACS file (required).
*/
int main(int argc, char** argv) {
  auto start = std::chrono::system_clock::now();

  signal(SIGINT, signalHandler);

  std::string inputPath;
  bool showHelp = false;

  // simple manual scan for launcher options (optree options only expose
  // long flags, so -i/-h are handled here).
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      showHelp = true;
    } else if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
      inputPath = argv[++i];
    }
  }

  d4::OptionMaxTMethod options;
  d4::OptionRegistry registry;
  options.registerTo(registry);
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

  if (!fs::exists(inputPath)) {
    std::cerr << "ERROR! Input file does not exist: " << inputPath << "\n";
    return 1;
  }

  parser::Formula formula;
  parser::ParserDimacs parserDimacs;
  parserDimacs.parse_DIMACS(inputPath, formula);

  std::cout << "c [INITIAL INPUT] \033[4m\033[32mStatistics about the input "
               "formula\033[0m\n";
  std::cout << "c [INITIAL INPUT] nbVar(" << formula.nbVar << ") nbClauses("
            << formula.clauses.size() << ")\n";
  std::cout << "c\n";

  maxT(options, formula);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "c [MAX-T] Elapsed time: " << elapsed_seconds.count()
            << " seconds\n";

  return EXIT_SUCCESS;
}  // main
