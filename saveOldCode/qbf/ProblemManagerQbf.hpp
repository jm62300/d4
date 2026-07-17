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

#include "../CnfMatrix.hpp"
#include "../ProblemManager.hpp"
#include "../ProblemTypes.hpp"
#include "src/solvers/WrapperSolver.hpp"

namespace d4 {

struct Block {
  bool isUniversal;
  std::vector<Var> variables;
};

class ProblemManagerQbf : public ProblemManager, public CnfMatrix {
 private:
  std::vector<Block> m_qblocks;

 public:
  ProblemManagerQbf();

  ProblemManagerQbf(const std::string &nameFile);
  ProblemManagerQbf(const int fd, bool keepOpen = false);

  ~ProblemManagerQbf();
  void display(std::ostream &out) override;

  inline void setBlocks(std::vector<Block> qblocks) { m_qblocks = qblocks; }
  inline std::vector<Block> &getQBlocks() { return m_qblocks; }

  /**
   * @brief Print out some statistic about the problem. Each line will start
   * with the string startLine given in parameter.
   *
   * @param[in] out is the stream where the messages are redirected.
   * @param[in] startLine is a string s.t. each line will start with this
   * string.
   */
  void displayStat(std::ostream &out, std::string startLine) override;

  ProblemManager *getUnsatProblem() override;
  ProblemManager *getConditionedFormula(std::vector<Lit> &units) override;

  inline ProblemInputType getProblemType() const override { return PB_QBF; }
  inline ProblemManager *translate(const ProblemTranslateType &t) override;
};
}  // namespace d4
