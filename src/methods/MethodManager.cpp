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

#include "MethodManager.hpp"

#include <boost/multiprecision/cpp_int.hpp>
#include <boost/multiprecision/gmp.hpp>

#include "DpllStyleMethod.hpp"
#include "Erosion.hpp"
#include "ExistRandomExist.hpp"
#include "MaxSharpSAT.hpp"
#include "MinSharpSAT.hpp"
#include "OperationManager.hpp"
#include "ProjMCMethod.hpp"
#include "src/configurations/ConfigurationDpllStyleMethod.hpp"
#include "src/exceptions/BadBehaviourException.hpp"
#include "src/exceptions/FactoryException.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/options/methods/OptionMethodManager.hpp"
#include "src/options/preprocs/OptionPreprocManager.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
namespace mpz = boost::multiprecision;

/**
   Consider the option in order to generate an instance of the wanted method.

   @param[in] vm, the map of option.
   @param[in] out, the stream where are print the information.
   @param[in] meth, the method we search to construct.
   @param[in] precision, the precision for the bignum.
   @param[in] isFloat, decide if the binum are float or int.
   @param[in] out, the stream where are printed the information.

   \return a method manager.
 */
MethodManager *MethodManager::makeMethodManager(po::variables_map &vm,
                                                ProblemManager *problem,
                                                const Configuration &config,
                                                std::ostream &out) {
  out << "c [METHOD MANAGER]\n";
  boost::multiprecision::mpf_float::default_precision(config.precision);

  if (config.methodName != METH_EROSION) {
    ProblemManager *runProblem = runPreproc(
        OptionPreprocManager(config.configurationPreproc), problem, out);

    bool isFloat = config.isFloat;
    displayInfoVariables(runProblem, out);
    for (unsigned i = 0; !isFloat && i < problem->getNbVar(); i++) {
      Lit l = Lit::makeLitTrue(i);
      if (problem->getWeightLit(l) != 1 || problem->getWeightLit(~l) != 1) {
        isFloat = 1;
        out << "c [METHOD MANAGER] Change to float mode!\n";
      }
    }

    if (config.methodName == METH_COUNTING || config.methodName == METH_DDNNF) {
      OptionDpllStyleMethod options(
          static_cast<const ConfigurationDpllStyleMethod &>(config));

      if (config.methodName == METH_COUNTING) {
        if (!isFloat)
          return new DpllStyleMethod<mpz::mpz_int, mpz::mpz_int>(
              options, runProblem, out);
        else
          return new DpllStyleMethod<mpz::mpf_float, mpz::mpf_float>(
              options, runProblem, out);
      }

      // compiler.
      if (!isFloat)
        return new DpllStyleMethod<mpz::mpz_int, Node<mpz::mpz_int> *>(
            options, runProblem, out);
      else
        return new DpllStyleMethod<mpz::mpf_float, Node<mpz::mpf_float> *>(
            options, runProblem, out);
    }

    if (config.methodName == METH_PROJ_MC) {
      if (!isFloat)
        return new ProjMCMethod<mpz::mpz_int>(vm, isFloat, runProblem);
      return new ProjMCMethod<mpz::mpf_float>(vm, isFloat, runProblem);
    }

    if (config.methodName == METH_MAX_SHARP) {
      if (!isFloat)
        return new MaxSharpSAT<mpz::mpz_int>(vm, "max#sat", isFloat, runProblem,
                                             out);
      return new MaxSharpSAT<mpz::mpf_float>(vm, "max#sat", isFloat, runProblem,
                                             out);
    }

    if (config.methodName == METH_ERE)
      return new ExistRandomExist<mpz::mpf_float>(vm, "ere", runProblem, out);

    if (config.methodName == METH_MIN_SHARP) {
      if (!isFloat)
        return new MinSharpSAT<mpz::mpz_int>(vm, "min#sat", isFloat, runProblem,
                                             out);
      return new MinSharpSAT<mpz::mpf_float>(vm, "min#sat", isFloat, runProblem,
                                             out);
    }
  } else {
    return new Erosion<mpz::mpz_int>(vm, config.isFloat,
                                     new ProblemManagerErosionCnf(problem));
  }

  throw(FactoryException("Cannot create a MethodManager", __FILE__, __LINE__));
}  // makeMethodManager

/**
 * @brief Display the projected variables in order.
 * @param[in] selected The list of projected variables.
 * @param[in] out The stream where is printed out the information.
 */
void MethodManager::displayInfoVariables(ProblemManager *problem,
                                         std::ostream &out) {
  std::vector<Var> &selected = problem->getSelectedVar();
  if (selected.size()) {
    out << "c\nc [PROJECTED VARIABLES] list: ";
    std::sort(selected.begin(), selected.end());
    for (auto v : selected) out << v << " ";
    out << "\nc\n";
  }

  std::vector<Var> &maxVar = problem->getMaxVar();
  if (maxVar.size()) {
    out << "c\nc [MAX VARIABLES] list: ";
    std::sort(maxVar.begin(), maxVar.end());
    for (auto v : maxVar) out << v << " ";
    out << "\nc\n";
  }

  std::vector<Var> &indVar = problem->getIndVar();
  if (indVar.size()) {
    out << "c\nc [IND VARIABLES] list: ";
    std::sort(indVar.begin(), indVar.end());
    for (auto v : indVar) out << v << " ";
    out << "\nc\n";
  }
}  // displayInfoProjected

/**
 * @brief Run the preproc method before constructing the method.
 *
 * @param[in] optionPreproc is the option list.
 * @param[in] initProblem is the input problem we want to preproc.
 * @param[in] out is the stream where will be printed out the log.
 * @param[out] lastBreath information collected when the preproc has done its
 * job.
 */
ProblemManager *MethodManager::runPreproc(
    const OptionPreprocManager &optionPreproc, ProblemManager *initProblem,
    std::ostream &out) {
  PreprocManager *preproc =
      PreprocManager::makePreprocManager(optionPreproc, out);
  assert(preproc);
  ProblemManager *problem = preproc->run(initProblem, optionPreproc.timeout);
  out << "c [MAIN PREPROCESSED INPUT] \033[4m\033[32mStatistics about the "
         "preprocessed formula\033[0m\n";
  problem->displayStat(out, "c [PREPROCESSED INPUT] ");
  out << "c\n";
  assert(problem);
  delete preproc;  // the preproc won't be used.

  return problem;
}  // runPreproc

}  // namespace d4
