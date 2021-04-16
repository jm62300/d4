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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <boost/program_options.hpp>
#include <iostream>

#include "MethodManager.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"

namespace d4 {
namespace po = boost::program_options;
template <class T, class U> class MixedProjMCMethod : public MethodManager {
public:
  /**
   * @brief Constructor.

      @param[in] vm, the list of options.
  */
  MixedProjMCMethod(po::variables_map &vm, std::string &meth, bool isFloat,
                    ProblemManager *initProblem, std::ostream &out) {
    ProblemManagerCnf *cnf = static_cast<ProblemManagerCnf *>(initProblem);

    std::cout << "Hello world\n";
  } // constructor

  /**
   * @brief Call the run function of the selected approach.
   * @param vm The option list.
   */
  void run(po::variables_map &vm) { std::cout << "Run\n"; } // run
};
} // namespace d4
