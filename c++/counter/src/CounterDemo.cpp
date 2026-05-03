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
 public:
  // Required by std::default_initializable
  BoostMpzSemiring() = default;

  // Required by your SemiringPolicy constructor constraint
  BoostMpzSemiring(unsigned nbVar,
                   const std::map<Lit, std::string>& literalWeights) {}

  // --- In-Place Multiplication ---
  mpz::mpz_int& mul(mpz::mpz_int& a, const mpz::mpz_int& b) const {
    a *= b;
    return a;
  }

  // --- In-Place Standard Binary Add ---
  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b) const {
    a += b;
    return a;
  }

  // --- In-Place Binary Add with Smoothing ---
  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b,
                    const std::vector<Lit>& units) const {
    a += b;
    return mul(a, one(units));  // mul modifies 'a' in place and returns it!
  }

  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b,
                    const std::vector<Var>& free_vars) const {
    a += b;
    return mul(a, one(free_vars));
  }

  mpz::mpz_int& add(mpz::mpz_int& a, const mpz::mpz_int& b,
                    const std::vector<Lit>& units,
                    const std::vector<Var>& free_vars) const {
    a += b;
    return mul(a, one(units, free_vars));
  }

  // --- In-Place Unary Adds (Smoothing a single branch) ---
  mpz::mpz_int& add(mpz::mpz_int& a, const std::vector<Lit>& units) const {
    return mul(a, one(units));
  }

  mpz::mpz_int& add(mpz::mpz_int& a, const std::vector<Var>& free_vars) const {
    return mul(a, one(free_vars));
  }

  mpz::mpz_int& add(mpz::mpz_int& a, const std::vector<Lit>& units,
                    const std::vector<Var>& free_vars) const {
    return mul(a, one(units, free_vars));
  }

  // Identities& Context - Aware Leaf Evaluation-- -

  mpz::mpz_int zero() const { return mpz::mpz_int(0); }

  mpz::mpz_int one() const { return mpz::mpz_int(1); }

  mpz::mpz_int one(const std::vector<Lit>& units) const {
    return mpz::mpz_int(1);
  }

  mpz::mpz_int one(const std::vector<Var>& free_vars) const {
    return mpz::mpz_int(1) << free_vars.size();
  }

  mpz::mpz_int one(const std::vector<Lit>& units,
                   const std::vector<Var>& free_vars) const {
    return mpz::mpz_int(1) << free_vars.size();
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