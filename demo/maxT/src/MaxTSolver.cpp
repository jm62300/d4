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

#include "MaxTSolver.hpp"

#include <signal.h>

#include <cassert>

#include "ParseOption.hpp"
#include "src/configurations/ConfigurationMaxTMethod.hpp"
#include "src/methods/MaxT.hpp"
#include "src/methods/MethodManager.hpp"
#include "src/options/methods/OptionMaxTMethod.hpp"

extern d4::MethodManager *methodRun;

using namespace d4;

template <typename T>
void maxT(const OptionMaxTMethod &options, ProblemManager *problem) {
  MaxT<T> *maxT = new MaxT<T>(options, problem, std::cout);

  methodRun = maxT;
  maxT->run();
  methodRun = nullptr;
  delete maxT;
}  // count

/**
 * @brief couterDemo implementation.
 */
void maxT(const po::variables_map &vm, ProblemManager *problem) {
  // get the configuration.
  ConfigurationMaxTMethod config;

  bool isFloat = problem->isFloat();
  MethodManager::displayInfoVariables(problem, std::cout);

  // init the options.
  config.threshold = -1;
  OptionMaxTMethod options(config);

  if (!isFloat)
    maxT<mpz::mpz_int>(options, problem);
  else
    maxT<mpz::mpf_float>(options, problem);
}  // counterDemo