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
#include <unordered_set>

#include "Counter.hpp"
#include "OptionCounter.hpp"
#include "ParserCircuit.hpp"
#include "ParserDimacs.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/preproc/PreprocManager.hpp"
#include <arjun/arjun.h>
#include <gmp.h>

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
  // scope below all variables (e.g. --preproc.project-on-input narrows it to
  // the bipartition input set). When the scope still covers every variable this
  // clears the quantification so standard model counting is used.
  if (formula.quantifications.empty()) formula.quantifications.emplace_back();
  if (projected.size() < formula.nbVar)
    formula.quantifications[0] = projected;
  else
    formula.quantifications[0].clear();
}  // runPreproc

// Run Arjun preprocessing on the formula in place.
static void runArjunPreproc(parser::Formula& formula,
                            const d4::OptionCounter& optionCounter,
                            const bipe::OptionPreproc& optionPreproc) {
  // Check if we have variables to project/minimize
  std::vector<uint32_t> projected;
  if (formula.quantifications[0].size()) {
    for (int v : formula.quantifications[0]) {
      projected.push_back(v - 1);
    }
  } else {
    for (unsigned i = 0; i < formula.nbVar; i++) {
      projected.push_back(i);
    }
  }

  bool all_indep = (projected.size() == formula.nbVar);

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

  // Create Arjun SimplifiedCNF and populate it
  ArjunNS::FGenMpq fg;
  ArjunNS::SimplifiedCNF cnf(&fg);
  cnf.set_need_aig();
  cnf.new_vars(formula.nbVar);
  cnf.set_weighted(true);

  for (const auto& cl : formula.clauses) {
    std::vector<CMSat::Lit> c;
    for (int lit : cl) {
      uint32_t var = std::abs(lit) - 1;
      bool sign = (lit < 0);
      c.push_back(CMSat::Lit(var, sign));
    }
    cnf.add_clause(c);
  }

  cnf.set_sampl_vars(projected);

  // Set weights for protected variables to prevent elimination
  ArjunNS::FMpq w_protect(mpq_class(1, 3));
  for (int v : varProtected) {
    cnf.set_lit_weight(CMSat::Lit(v - 1, false), w_protect);
  }

  // Preprocess with Arjun
  ArjunNS::Arjun arjun;
  arjun.set_verb(optionCounter.verbosity.get());
  arjun.set_seed(0);

  // Map optionPreproc settings to Arjun
  arjun.set_probe_based(optionPreproc.optionEliminator.probing.get());
  arjun.set_bve_pre_simplify(optionPreproc.optionEliminator.bva.get());
  arjun.set_distill(optionPreproc.optionEliminator.oracleVivif.get());
  arjun.set_gauss_jordan(optionPreproc.optionEliminator.bvaStructured.get());
  arjun.set_xor_gates_based(optionPreproc.optionEliminator.ternaryRes.get());

  // Run backbone first
  arjun.standalone_backbone(cnf);

  // Run minimization
  arjun.standalone_minimize_indep(cnf, all_indep);
  // Propagate back to formula
  formula.clauses.clear();

  for (const auto& cl : cnf.get_clauses()) {
    std::vector<int> c;
    for (const auto& lit : cl) {
      int var = lit.var() + 1;
      int human_lit = lit.sign() ? -var : var;
      c.push_back(human_lit);
    }
    formula.clauses.push_back(c);
  }

  for (const auto& cl : cnf.get_red_clauses()) {
    std::vector<int> c;
    for (const auto& lit : cl) {
      int var = lit.var() + 1;
      int human_lit = lit.sign() ? -var : var;
      c.push_back(human_lit);
    }
    formula.clauses.push_back(c);
  }

  // Reflect the (possibly narrowed) projected set back into the formula
  if (formula.quantifications.empty()) formula.quantifications.emplace_back();
  std::vector<int> newProjected;
  if (formula.quantifications[0].empty()) {
    for (unsigned i = 0; i < formula.nbVar; i++) {
      if (!cnf.defined(i)) {
        newProjected.push_back(i + 1);
      }
    }
  } else {
    std::unordered_set<int> origProjSet(formula.quantifications[0].begin(), formula.quantifications[0].end());
    for (unsigned i = 0; i < formula.nbVar; i++) {
      if (origProjSet.count(i + 1) && !cnf.defined(i)) {
        newProjected.push_back(i + 1);
      }
    }
  }

  if (newProjected.size() < formula.nbVar)
    formula.quantifications[0] = newProjected;
  else
    formula.quantifications[0].clear();
}


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
    if (optionCounter.preprocEngine.get() == "arjun") {
      if (!formula.quantifications.empty() && formula.quantifications[0].size() > 0) {
        std::cout << "c [PREPROC] Projected model counting detected. Falling back to bipe engine for correctness.\n";
        runPreproc(formula, optionPreproc);
      } else {
        runArjunPreproc(formula, optionCounter, optionPreproc);
      }
    } else {
      runPreproc(formula, optionPreproc);
    }
  }

  counter(options, optionCounter, formula);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "c [COUNTER] Elapsed time: " << elapsed.count() << " seconds\n";

  return EXIT_SUCCESS;
}  // main