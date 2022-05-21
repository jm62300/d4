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
#pragma once

#include <boost/program_options.hpp>
#include <vector>

#include "../PreprocManager.hpp"
#include "3rdParty/bipe/srcBipe/methods/Method.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
namespace po = boost::program_options;
class PreprocBackboneCnf : public PreprocManager {
 private:
  WrapperSolver *ws;
  bipe::Method *m_isRunning = NULL;

 public:
  PreprocBackboneCnf(po::variables_map &vm, std::ostream &out);
  ~PreprocBackboneCnf();
  virtual ProblemManager *run(ProblemManager *pin,
                              LastBreathPreproc &lastBreath,
                              unsigned timeout) override;
};
}  // namespace d4
