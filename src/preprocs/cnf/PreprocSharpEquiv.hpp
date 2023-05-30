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
#include "3rdParty/bipe/src/bipartition/methods/Method.hpp"
#include "3rdParty/bipe/src/eliminator/Eliminator.hpp"
#include "3rdParty/bipe/src/reducer/Method.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/problem/cnf/ProblemManagerCnf.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {
namespace po = boost::program_options;
class PreprocSharpEquiv : public PreprocManager {
 private:
  WrapperSolver *ws;
  std::string m_method;
  int m_nbIteration;
  bool m_isInterrupted = false;

  /**
   * @brief Compute the bipartition.
   *
   * @param pcnf is the CNF we want to compute the bipartition (and some
   * gates).
   * @param[out] units stores the unit literals.
   * @param[out] input stores the input variables.
   * @param[out] output is the set of output variables.
   * @param[out] gates stores the extracted gates.
   * @param timeout is the timeout for computing the bipartition.
   */
  void computeBipartition(ProblemManagerCnf &pcnf, std::vector<Lit> &units,
                          std::vector<bipe::Var> &input,
                          std::vector<bipe::Var> &output,
                          std::vector<bipe::Gate> &gates, unsigned timeout);

  /**eliminator
   * @brief Apply distillation to a given set of clauses (with units).
   *
   * @param clauses is the set of clauses.
   * @param[out] units is the set of unit clauses.
   * @param[out] isUnit gives the variables that are unit.
   * @param nbVar is the number of variables.
   * @param[out] resClauses is the simplified formula (without the units).
   *
   * \return true if the formula has been modified.
   */
  bool applyDistillation(std::vector<std::vector<Lit>> &clauses,
                         std::vector<Lit> &units, std::vector<bool> &isUnit,
                         unsigned nbVar,
                         std::vector<std::vector<Lit>> &resClauses);

  /**
   * @brief Try to eliminate some variables.
   *
   * @param clauses is the CNF formula.
   * @param units is the set of unit literals.
   * @param isUnit gives if a variable is unit or not.
   * @param nbVar is the number of variables.
   * @param input are the input variables.
   * @param dac is the set of gates.
   * @param eliminated are the variables removed.
   * @param resClauses is the resulting CNF.
   * @param limitNbClauses give the maximum number of clause we can have in the
   * result.
   * @return true if we remove some variables, false otherwise.
   */
  bool applyElimination(std::vector<std::vector<Lit>> &clauses,
                        std::vector<Lit> &units, std::vector<bool> &isUnit,
                        unsigned nbVar, std::vector<bipe::Var> &input,
                        std::vector<bipe::Gate> &dac,
                        std::vector<bipe::Lit> &eliminated,
                        std::vector<std::vector<Lit>> &resClauses,
                        unsigned limitNbClauses);

 public:
  PreprocSharpEquiv(po::variables_map &vm, std::string &method, int nbIteration,
                    std::ostream &out);
  ~PreprocSharpEquiv();
  virtual ProblemManager *run(ProblemManager *pin,
                              LastBreathPreproc &lastBreath,
                              unsigned timeout) override;

  /**
   * @brief Stop.
   *
   */
  inline void interrupt() { m_isInterrupted = true; }  // interrupt
};
}  // namespace d4
