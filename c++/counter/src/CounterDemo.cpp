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

#include "CounterDemo.hpp"

#include <signal.h>

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/integer.hpp>
#include <cassert>

#include "ParseOption.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/methods/DpllStyleMethod.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"

extern d4::MethodManager* methodRun;

using namespace d4;

#include <boost/multiprecision/gmp.hpp>
#include <map>
#include <string>
#include <vector>

// Your namespace alias
namespace mpz = boost::multiprecision;

class BoostMpzSemiring {
 private:
  std::vector<Var> m_decisionVars;

  // Storing the parsed weights as native Boost multiprecision integers
  std::map<Lit, mpz::mpz_int> m_weights;

  /**
   * @brief Safely fetches a literal's weight, defaulting to 1 (identity).
   */
  mpz::mpz_int getWeight(Lit l) const {
    auto it = m_weights.find(l);
    if (it != m_weights.end()) {
      return it->second;
    }
    return mpz::mpz_int(1);
  }

  /**
   * @brief Calculates the smoothed weight for a free variable: W(v) + W(-v)
   */
  mpz::mpz_int getFreeVarWeight(Var v) const {
    // Assuming Lit::makeLit(var, sign) where false = positive lit, true =
    // negative lit
    mpz::mpz_int pos_weight = getWeight(Lit::makeLit(v, false));
    mpz::mpz_int neg_weight = getWeight(Lit::makeLit(v, true));
    return pos_weight + neg_weight;
  }

 public:
  BoostMpzSemiring() = default;

  // --- 1. The Concept-Mandated Constructor ---
  BoostMpzSemiring(const std::vector<Var>& decisionVars,
                   const std::map<Lit, std::string>& literalWeights)
      : m_decisionVars(decisionVars) {
    // Boost's mpz_int parses strings instantly in its constructor
    for (const auto& [lit, str_weight] : literalWeights) {
      m_weights[lit] = mpz::mpz_int(str_weight);
    }
  }

  // --- 2. Addition (OR Nodes) ---
  mpz::mpz_int add(const mpz::mpz_int& a, const mpz::mpz_int& b) const {
    return a + b;
  }

  mpz::mpz_int add(const mpz::mpz_int& a, const mpz::mpz_int& b,
                   const std::vector<Lit>& units) const {
    return mul(add(a, b), one(units));
  }

  mpz::mpz_int add(const mpz::mpz_int& b, const std::vector<Lit>& units,
                   const std::vector<Var>& free_vars) const {
    return b;
  }

  mpz::mpz_int add(const mpz::mpz_int& a, const mpz::mpz_int& b,
                   const std::vector<Var>& free_vars) const {
    return mul(add(a, b), one(free_vars));
  }

  mpz::mpz_int add(const mpz::mpz_int& a, const mpz::mpz_int& b,
                   const std::vector<Lit>& units,
                   const std::vector<Var>& free_vars) const {
    return mul(add(a, b), one(units, free_vars));
  }

  // --- 3. Multiplication (AND Nodes) ---
  mpz::mpz_int mul(const mpz::mpz_int& a, const mpz::mpz_int& b) const {
    return a * b;
  }

  // --- 4. Identities & Context-Aware Leaf Evaluation ---
  mpz::mpz_int zero() const { return mpz::mpz_int(0); }

  mpz::mpz_int one() const { return mpz::mpz_int(1); }

  mpz::mpz_int one(const std::vector<Lit>& units) const {
    mpz::mpz_int res(1);
    for (Lit l : units) {
      res *= getWeight(l);
    }
    return res;
  }

  mpz::mpz_int one(const std::vector<Var>& free_vars) const {
    mpz::mpz_int res(1);
    for (Var v : free_vars) {
      res *= getFreeVarWeight(v);
    }
    return res;
  }

  mpz::mpz_int one(const std::vector<Lit>& units,
                   const std::vector<Var>& free_vars) const {
    return mul(one(units), one(free_vars));
  }

  // --- 5. Presets (Required by Policy) ---
  mpz::mpz_int presetSum(int /* gate_id */) const { return mpz::mpz_int(0); }

  mpz::mpz_int presetMul(int /* gate_id */) const { return mpz::mpz_int(1); }
};

template <typename T>
void countModels(const OptionDpllStyleMethod& options,
                 const ProblemManager& problem, const std::string& format,
                 const std::string& outFormat, bool isFloat) {
  std::cout << "c [FORMAT] Input/Output format:" << " output-symbol(" << format
            << ")" << " output-format(" << outFormat << ")" << " is-float("
            << isFloat << ")\n";

  DpllStyleMethod<T, BoostMpzSemiring>* counter =
      new DpllStyleMethod<T, BoostMpzSemiring>(options, problem, std::cout);

  methodRun = counter;
  T result = counter->run();

  if (outFormat == "competition") {
    boost::multiprecision::mpf_float::default_precision(128);
    std::cout.precision(
        std::numeric_limits<boost::multiprecision::cpp_dec_float_50>::digits10);

    if (result == 0) {
      std::cout << "s UNSATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate -inf\n";
      std::cout << "c s exact quadruple int 0\n";
    } else {
      std::cout << "s SATISFIABLE\n";
      std::cout << "c " << format << "\n";
      std::cout << "c s log10-estimate "
                << boost::multiprecision::log10(
                       boost::multiprecision::cpp_dec_float_100(result))
                << "\n";
      if (isFloat)
        std::cout << "c s exact quadruple int " << result << "\n";
      else
        std::cout << "c s exact arb int " << result << "\n";
    }
    exit(0);  // stop faster than cleaning the memory!
  } else {
    assert(outFormat == "classic");
    std::cout << format << " ";
    std::cout << std::fixed << std::setprecision(50) << result << "\n";
  }

  methodRun = nullptr;
  delete counter;
}  // count

/**
 * @brief couterDemo implementation.
 */
void counterDemo(const po::variables_map& vm, const ProblemManager& problem) {
  // get the configuration.
  ConfigurationDpllStyleMethod config;

  config.methodName = d4::MethodNameManager::getMethodName("counting");

  config.inputName = vm["input"].as<std::string>();
  config.problemInputType = d4::ProblemInputTypeManager::getInputType(
      vm["input-type"].as<std::string>());

  config.cache = parseCacheConfiguration(vm);
  config.branchingHeuristic = parseBranchingHeuristicConfiguration(vm);
  config.solver.solverName =
      d4::SolverNameManager::getSolverName(vm["solver"].as<std::string>());

  config.spec.specUpdateType = d4::SpecUpdateManager::getSpecUpdate(
      vm["occurrence-manager"].as<std::string>());
  config.spec.removeGates = vm["remove-gates"].as<bool>();

  config.exploitModel = vm["exploit-model-activated"].as<bool>();
  config.operationType = d4::OperationTypeManager::getOperatorType("counting");

  // init the options.
  if (config.cache.clauseRepresentation == CACHE_INDEX)
    config.spec.needFastNotSatisfied = true;
  OptionDpllStyleMethod options(config);

  // construct and call the counter regarding if it is MC or WMC.
  std::string format = vm["keyword-output-format-solution"].as<std::string>();
  std::string outFormat = vm["output-format"].as<std::string>();

  countModels<mpz::mpz_int>(options, problem, format, outFormat, false);
  //  } else
  //    countModels<mpz::mpf_float>(options, problem, format, outFormat, true);
}  // counterDemo