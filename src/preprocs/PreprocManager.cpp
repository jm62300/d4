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
#include "PreprocManager.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <csignal>

#include "circuit/PreprocCnfFromCircuit.hpp"
#include "cnf/PreprocBackboneCnf.hpp"
#include "cnf/PreprocBasicCnf.hpp"
#include "cnf/PreprocReducer.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {

PreprocManager *PreprocManager::s_isRunning = nullptr;

/**
 * @brief Preproc factory.
 *
 * @param vm gives the options.
 * @param out is the stream where are printed out the logs.
 * @return a preproc.
 */
PreprocManager *PreprocManager::makePreprocManager(po::variables_map &vm,
                                                   std::ostream &out) {
  std::string meth = vm["preproc"].as<std::string>();
  std::string inputType = vm["input-type"].as<std::string>();
  int timeout = vm["preproc-timeout"].as<int>();

  out << "c [PREPROC] Method: " << meth << " " << inputType << "\n";
  out << "c [PREPROC] Timeout: " << timeout << "\n";

  PreprocManager *ret = nullptr;
  if (inputType == "cnf" || inputType == "dimacs") {
    if (meth == "basic")
      ret = new PreprocBasicCnf(vm, out);
    else if (meth == "backbone")
      ret = new PreprocBackboneCnf(vm, out);
    else if (meth == "vivification" || meth == "occElimination" ||
             meth == "combinaison")
      ret = new PreprocReducer(vm, meth,
                               vm["preproc-reducer-iteration"].as<int>(), out);
  }

  if (inputType == "circuit") {
    ret = new PreprocCnfFromCircuit(vm, out);
  }

  if (!ret)
    throw(
        FactoryException("Cannot create a PreprocManager", __FILE__, __LINE__));

  if (timeout != -1) {
    out << "c [PREPROC] Set the signal handler\n";
    assert(!s_isRunning);
    s_isRunning = ret;
    signal(SIGALRM, PreprocManager::static_handler);
    alarm(timeout);
  }

  return ret;
}  // makePreprocManager

}  // namespace d4
