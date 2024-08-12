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

#include "ProblemManagerCircuit.hpp"

#include "../cnf/ProblemManagerCnf.hpp"
#include "ParserCircuit.hpp"
#include "src/problem/ProblemManager.hpp"

namespace d4 {
/**
   Constructor.

   @param[in] nameFile, parse the instance from a file
 */
ProblemManagerCircuit::ProblemManagerCircuit(const std::string &nameFile) {
  ParserCircuit parser;
  m_nbVar = parser.parse_circuit(nameFile, this);
  assert(m_weightLit.size() == ((m_nbVar + 1) << 1));
  m_weightVar.resize(m_nbVar + 1, 0);
  for (unsigned i = 0; i <= m_nbVar; i++)
    m_weightVar[i] = m_weightLit[i << 1] + m_weightLit[(i << 1) + 1];

  m_order.resize(m_nbVar + 1);
  for (unsigned i = 0; i < m_order.size(); i++) m_order[i] = i;
}  // constructor

/**
   Constructor.
   Construct an empty formula.
 */
ProblemManagerCircuit::ProblemManagerCircuit() { m_nbVar = 0; }  // constructor

/**
 * @brief Construct a new Problem Manager Bc:: Problem Manager Bc object
 *
 * @param problem a problem manager object.
 */
ProblemManagerCircuit::ProblemManagerCircuit(ProblemManager *problem) {
  m_nbVar = problem->getNbVar();
  m_weightLit = problem->getWeightLit();
  m_weightVar = problem->getWeightVar();
  m_selected = problem->getSelectedVar();
  m_maxVar = problem->getMaxVar();
  m_indVar = problem->getIndVar();
  m_isUnsat = false;

  m_order.resize(m_nbVar + 1);
  for (unsigned i = 0; i < m_order.size(); i++) m_order[i] = i;
}  // constructor

/**
 * @brief Construct a new Problem Manager Bc:: Problem Manager Bc object
 *
 * @param nbVar, the number of variables.
 * @param weightLit, the weights associate with the literals.
 * @param weightVar, the weights associate with the variables (sum of weight
   of the lit)
 * @param selected, the projected variables.
 */
ProblemManagerCircuit::ProblemManagerCircuit(
    int nbVar, std::vector<mpz::mpf_float> &weightLit,
    std::vector<mpz::mpf_float> &weightVar, std::vector<Var> &selected) {
  m_nbVar = nbVar;
  m_weightLit = weightLit;
  m_weightVar = weightVar;
  m_selected = selected;
  m_isUnsat = false;
}  // constructor

/**
 * @brief Construct a new Problem Manager Bc:: Problem Manager Bc object
 *
 * @param nbVar, the number of variables.
 * @param weightLit, the weights associate with the literals.
 * @param weightVar, the weights associate with the variables (sum of weight
   of the lit)
 * @param selected, the projected variables.
 * @param maxVar is the set of existential variables.
 * @param indVar is the set of randomized variables.
 */
ProblemManagerCircuit::ProblemManagerCircuit(
    int nbVar, std::vector<mpz::mpf_float> &weightLit,
    std::vector<mpz::mpf_float> &weightVar, std::vector<Var> &selected,
    std::vector<Var> &maxVar, std::vector<Var> &indVar) {
  m_nbVar = nbVar;
  m_weightLit = weightLit;
  m_weightVar = weightVar;
  m_selected = selected;
  m_maxVar = maxVar;
  m_indVar = indVar;
  m_isUnsat = false;
}  // constructor

/**
   Destructor.
 */
ProblemManagerCircuit::~ProblemManagerCircuit() {
  m_gates.clear();
  m_nbVar = 0;
}  // destructor

/**
 * @brief Get the Unsat ProblemManager object.
 *
 * @return an unsatisfiable problem.
 */
ProblemManager *ProblemManagerCircuit::getUnsatProblem() {
  ProblemManagerCircuit *ret = new ProblemManagerCircuit(this);
  ret->m_isUnsat = true;

  std::vector<Lit> &true_lits = ret->getTrueLiterals();
  true_lits.push_back({Lit::makeLit(1, false)});
  true_lits.push_back({Lit::makeLit(1, true)});

  return ret;
}  // getUnsatProblem

/**
 * @brief Simplify the formula by unit propagation and return the resulting CNF
 * formula.
 *
 * @param units is the set of unit literals we want to condition with.
 * @return the simplified formula.
 */
ProblemManager *ProblemManagerCircuit::getConditionedFormula(
    std::vector<Lit> &units) {
  ProblemManagerCircuit *ret = new ProblemManagerCircuit(this);

  ret->m_gates = this->m_gates;
  ret->m_true_lits = this->m_true_lits;

  for (Lit l : units) {
    ret->m_true_lits.push_back(l);
  }

  return ret;
}  // getConditionedFormula

/**
 * @brief ProblemManagerCircuit::getInputVar implementation.
 */
void ProblemManagerCircuit::getInputVar(std::vector<Var> &outVars) {
  std::vector<bool> marked(getNbVar() + 1, true);
  for (auto &g : m_gates) marked[g.output.var()] = false;
  for (unsigned i = 1; i <= getNbVar(); i++)
    if (marked[i]) outVars.push_back(i);
}  // getInputVar

/**
   Display the problem.

   @param[out] out, the stream where the messages are redirected.
 */
void ProblemManagerCircuit::display(std::ostream &out) {
  out << "weight list: ";
  for (unsigned i = 1; i <= m_nbVar; i++) {
    Lit l = Lit::makeLit(i, false);
    out << i << "[" << m_weightVar[i] << "] ";
    out << l << "(" << m_weightLit[l.intern()] << ") ";
    out << ~l << "(" << m_weightLit[(~l).intern()] << ") ";
  }
  out << std::endl;

  out << "BC-S1.2 " << m_nbVar << " " << m_gates.size() << "\n";
  for (auto &gate : m_gates) {
    gate.display(out);
    out << std::endl;
  }

  out << "True literals: ";
  for (auto &Lit : m_true_lits) {
    out << Lit << " ";
  }
  std::cout << std::endl;
}  // diplay

/**
 * @brief Print out some statistic about the problem. Each line will start with
 * the string startLine given in parameter.
 *
 * @param[in] out is the stream where the messages are redirected.
 * @param[in] startLine is the string each line will start with.
 */
void ProblemManagerCircuit::displayStat(std::ostream &out,
                                        std::string startLine) {
  unsigned nbLits = 0;
  unsigned nbBin = 0;
  unsigned nbTer = 0;
  unsigned nbMoreThree = 0;

  for (auto &gate : m_gates) {
    nbLits += gate.input.size();
    if (gate.input.size() == 2) nbBin++;
    if (gate.input.size() == 3) nbTer++;
    if (gate.input.size() > 3) nbMoreThree++;
  }

  out << startLine << "Number of variables: " << m_nbVar << "\n";
  out << startLine << "Number of gates: " << m_gates.size() << "\n";
  out << startLine << "Number of binary gates: " << nbBin << "\n";
  out << startLine << "Number of ternary gates: " << nbTer << "\n";
  out << startLine << "Number of gates larger than 3: " << nbMoreThree << "\n";
  out << startLine << "Number of literals: " << nbLits << "\n";
}  // displaystat

/**
 * @brief ProblemManagerCircuit::tseitinEncoding implementation.
 */
void ProblemManagerCircuit::tseitinEncoding(
    std::vector<std::vector<Lit>> &clauses) {
  std::vector<Lit> cl;
  for (auto &g : m_gates) {
    switch (g.gate_type) {
      case BcGateType::AND:
        cl.clear();
        cl.push_back(g.output);
        for (auto l : g.input) {
          clauses.push_back({l, ~g.output});
          cl.push_back(~l);
        }
        clauses.push_back(cl);
        break;
      case BcGateType::OR:
        cl.clear();
        cl.push_back(~g.output);
        for (auto l : g.input) {
          clauses.push_back({~l, g.output});
          cl.push_back(l);
        }
        clauses.push_back(cl);
        break;
      case BcGateType::IDENTITY:
        assert(g.input.size() == 1);
        clauses.push_back({g.input[0], ~g.output});
        clauses.push_back({~g.input[0], g.output});
        break;
    }
  }

  for (auto &l : m_true_lits) clauses.push_back({l});
}  // tseintinEncoding

/**
 * @brief ProblemManagerCircuit::translate implementation.
 */
inline ProblemManager *ProblemManagerCircuit::translate(
    const ProblemTranslateType &t) {
  if (t == TRANSLATE_NONE) {
    // Create copy of this (since users may delete this after translation)
    ProblemManagerCircuit *ret = new ProblemManagerCircuit(this);
    ret->getTrueLiterals() = this->getTrueLiterals();
    ret->getGates() = this->getGates();
    return ret;
  }

  std::vector<Var> projectedVar;
  std::vector<std::vector<Lit>> clauses;
  tseitinEncoding(clauses);

  if (t == TRANSLATE_PCNF) {
    std::vector<bool> isInput(m_nbVar + 1, true);
    for (auto &g : m_gates) isInput[g.output.var()] = false;
    for (unsigned i = 1; i <= m_nbVar; i++)
      if (isInput[i]) projectedVar.push_back(i);
  }

  ProblemManagerCnf *ret =
      new ProblemManagerCnf(m_nbVar, m_weightLit, m_weightVar, projectedVar);
  ret->setClauses(clauses);

  return ret;
}  // translate

}  // namespace d4
