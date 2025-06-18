/*
 * d4
 * Copyright (C) 2024  Univ. Artois & CNRS & KU Leuven
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

#include "../ProblemManager.hpp"
#include "../ProblemTypes.hpp"

namespace d4 {

/**
 * @brief Different gate types.
 *
 * The IDENTITY gate has one input and simply passes
 * on the information. This could be used to represent
 * gates `x := y` or `x := -y`
 */
enum class BcGateType { AND, OR, IDENTITY };

struct BcGate {
  std::vector<Lit> input;
  Lit output;
  BcGateType gate_type;

  /**
   * @brief Print out the circuit.
   *
   * @param[out] out is the stream where the messages are redirected.
   */
  void display(std::ostream &out) {
    out << output;
    switch (gate_type) {
      case BcGateType::AND:
        out << " =(AND): ";
        break;
      case BcGateType::OR:
        out << " =(OR): ";
        break;
      case BcGateType::IDENTITY:
        out << " =(I): ";
        break;
      default:
        out << " =(UNKNOWN): ";
        break;
    }
    for (auto &l : input) out << l << " ";
  }
};

class ProblemManagerCircuit : public ProblemManager {
 private:
  std::vector<BcGate> m_gates;
  std::vector<Lit> m_true_lits;  // conjunction of output literals that must be
                                 // true. unsigned m_nbInputVars;

 public:
  ProblemManagerCircuit();

  ProblemManagerCircuit(int nbVar, std::vector<mpz::mpf_float> &weightLit,
                        std::vector<mpz::mpf_float> &weightVar,
                        std::vector<Var> &selected);

  ProblemManagerCircuit(int nbVar, std::vector<mpz::mpf_float> &weightLit,
                        std::vector<mpz::mpf_float> &weightVar,
                        std::vector<Var> &selected, std::vector<Var> &maxVar,
                        std::vector<Var> &indVar);

  ProblemManagerCircuit(ProblemManager *problem);

  ProblemManagerCircuit(const std::string &nameFile);

  ~ProblemManagerCircuit();
  void display(std::ostream &out) override;
  void displayStat(std::ostream &out, std::string startLine) override;
  ProblemManager *getUnsatProblem() override;
  ProblemManager *getConditionedFormula(std::vector<Lit> &units) override;

  /**
   * @brief Encode the formula using the tseitin encoding.
   *
   * @param clauses is the resulting CNF.
   */
  void tseitinEncoding(std::vector<std::vector<Lit>> &clauses);

  /**
   * @brief Collect the input variables.
   *
   * @param[out] outVars store the input vars.
   */
  void getInputVar(std::vector<Var> &outVars);

  std::vector<BcGate> &getGates() { return m_gates; }
  std::vector<Lit> &getTrueLiterals() { return m_true_lits; }
  inline ProblemInputType getProblemType() const override { return PB_CIRC; }
  ProblemManager *translate(const ProblemTranslateType &t) override;
};
}  // namespace d4
