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
#include "cnf/PreprocEquiv.hpp"
#include "cnf/PreprocReducer.hpp"
#include "cnf/PreprocSharpEquiv.hpp"
#include "src/exceptions/FactoryException.hpp"

namespace d4 {
void *PreprocManager::s_isRunning = nullptr;

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

  out << "c [PREPROC] Method: " << meth << " " << inputType << "\n";

  PreprocManager *ret = nullptr;
  if (inputType == "cnf" || inputType == "dimacs") {
    if (meth == "basic")
      ret = new PreprocBasicCnf(vm, out);
    else if (meth == "backbone")
      ret = new PreprocBackboneCnf(vm, out);
    else if (meth == "equiv")
      ret = new PreprocEquiv(vm, meth,
                             vm["preproc-reducer-iteration"].as<int>(), out);
    else if (meth == "sharp-equiv")
      ret = new PreprocSharpEquiv(
          vm, meth, vm["preproc-reducer-iteration"].as<int>(), out);
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

  return ret;
}  // makePreprocManager

}  // namespace d4
