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
#include "3rdParty/eliminator/srcEliminator/Eliminator.hpp"
#include "3rdParty/reducer/src/methods/Method.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
namespace po = boost::program_options;
class PreprocSharpEquiv : public PreprocManager {
 private:
  WrapperSolver *ws;
  std::string m_method;
  int m_nbIteration;
  bool m_isInterrupted = false;
  reducer::Method *m_isRunningReducer = NULL;
  bipe::Method *m_isRunningBackbone = NULL;

  /**
   * @brief Rewrite the DAC computed using bipe in another DAC ready to be used
   * with the eliminator library.
   *
   * @param gates is the DAC computed using bipe.
   * @param dac is the DAC that can be used with eliminator.
   */
  void expressDacInEliminatorFormat(std::vector<bipe::Gate> &gates,
                                    std::vector<eliminator::Gate> &dac);

 public:
  PreprocSharpEquiv(po::variables_map &vm, std::string &method, int nbIteration,
                    std::ostream &out);
  ~PreprocSharpEquiv();
  virtual ProblemManager *run(ProblemManager *pin,
                              LastBreathPreproc &lastBreath) override;

  /**
   * @brief Stop.
   *
   */
  inline void interrupt() {
    m_isInterrupted = true;
    if (m_isRunningReducer) {
      std::cout << "c [PREPROC #EQUIV] Stop the reducer because timeout\n";
      m_isRunningReducer->interrupt();
    }

    if (m_isRunningBackbone) {
      std::cout
          << "c [PREPROC #EQUIV] Stop the backbone extractor because timeout\n";
      m_isRunningBackbone->interrupt();
    }
  }  // interrupt
};
}  // namespace d4
