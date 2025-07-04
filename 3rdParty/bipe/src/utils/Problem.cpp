/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "Problem.hpp"

namespace bipe {

/**
   Constructor.
   Construct an empty formula.
 */
Problem::Problem() { m_nbVar = 0; }  // constructor

/**
 * @brief Construct a new Problem:: Problem object
 *
 * @param problem
 */
Problem::Problem(Problem *problem) : Problem(*problem) {}  // constructor.

/**
 * @brief Construct a new Problem Manager Cnf:: Problem Manager Cnf object
 *
 * @param problem, a problem manager object.
 */
Problem::Problem(Problem &problem) {
  m_nbVar = problem.getNbVar();
  m_weightLit = problem.getWeightLit();
  m_projected = problem.getProjectedVar();
  m_protected = problem.getProtectedVar();
}  // constructor

/**
 * @brief Construct a new Problem Manager Cnf:: Problem Manager Cnf object
 *
 * @param nbVar, the number of variables.
 * @param weightLit, the weights associate with the literals.
 * @param weightVar, the weights associate with the variables (sum of weight
   of the lit)
 * @param selected, the projected variables.
 */
Problem::Problem(int nbVar, std::vector<double> &weightLit,
                 std::vector<Var> &selected, std::vector<Var> &protect) {
  m_nbVar = nbVar;
  m_weightLit = weightLit;
  m_projected = selected;
  m_protected = protect;
}  // constructor

/**
   Destructor.
 */
Problem::~Problem() {
  m_clauses.clear();
  m_nbVar = 0;
}  // destructor

/**
 * @brief Get the Unsat ProblemManager object.
 *
 * @return an unsatisfiable problem.
 */
Problem *Problem::getUnsatProblem() {
  Problem *ret = new Problem(this);

  std::vector<Lit> cl;
  Lit l = Lit::makeLit(1, false);

  cl.push_back(l);
  ret->getClauses().push_back(cl);

  cl[0] = l.neg();
  ret->getClauses().push_back(cl);

  std::cout << "c [PROBLEM] create an UNSAT problem.\n";
  ret->setUnsat();

  return ret;
}  // getUnsatProblem

/**
 * @brief Simplify the formula by unit propagation and return the
 * resulting CNF formula.
 *
 * @param units is the set of unit literals we want to simplify the formula
 * with.
 * @return Problem* is the conditioned formula.
 */
Problem *Problem::getConditionedFormula(std::vector<Lit> &units) {
  Problem *ret = new Problem(this);

  std::vector<char> value(m_nbVar + 1, 0);
  for (auto l : units) {
    value[l.var()] = l.sign() + 1;
    ret->getClauses().push_back({l});
  }

  for (auto cl : m_clauses) {
    // get the simplified clause.
    std::vector<Lit> scl;
    bool isSAT = false;
    for (auto l : cl) {
      if (!value[l.var()]) scl.push_back(l);

      if ((isSAT = (l.sign() + 1 == value[l.var()]))) break;
    }

    // add the simplified clause if needed.
    if (!isSAT) ret->getClauses().push_back(scl);
  }

  return ret;
}  // getConditionedFormula

/**
   Display the problem.

   @param[out] out, the stream where the messages are redirected.
 */
void Problem::display(std::ostream &out) {
  out << "p cnf " << m_nbVar << " " << m_clauses.size() << "\n";

  if (m_projected.size()) {
    out << "c p show ";
    for (auto &v : m_projected) out << v << " ";
    out << "0\n";
  }

  bool isWeighted = false;
  for (auto &w : m_weightLit)
    if ((isWeighted = (w != 1))) break;

  for (auto cl : m_clauses) {
    for (auto &l : cl) out << l << " ";
    out << "0\n";
  }

  if (isWeighted) {
    for (int i = 1; i <= (int)m_nbVar; i++) {
      out << "c p weight " << i << " " << m_weightLit[i << 1] << " 0\n";
      out << "c p weight " << -i << " " << m_weightLit[1 + (i << 1)] << " 0\n";
    }
  }
}  // diplay

/**
   Print out some statistic about the problem. Each line will start with the
   string startLine given in parameter.

   @param[in] out, the stream where the messages are redirected.
   @param[in] startLine, each line will start with this string.
 */
void Problem::displayStat(std::ostream &out, std::string startLine) {
  unsigned nbLits = 0;
  unsigned nbBin = 0;
  unsigned nbTer = 0;
  unsigned nbMoreThree = 0;

  for (auto &c : m_clauses) {
    nbLits += c.size();
    if (c.size() == 2) nbBin++;
    if (c.size() == 3) nbTer++;
    if (c.size() > 3) nbMoreThree++;
  }

  bool isWeighted = false;
  for (auto &w : m_weightLit)
    if ((isWeighted = (w != 1))) break;

  out << startLine << "Weighted model counting: " << isWeighted << "\n";
  out << startLine << "Projected: " << (m_projected.size() != 0) << "\n";
  out << startLine << "Number of variables: " << m_nbVar << "\n";
  out << startLine << "Number of clauses: " << m_clauses.size() << "\n";
  out << startLine << "Number of binary clauses: " << nbBin << "\n";
  out << startLine << "Number of ternary clauses: " << nbTer << "\n";
  out << startLine << "Number of clauses larger than 3: " << nbMoreThree
      << "\n";
  out << startLine << "Number of literals: " << nbLits << "\n";
}  // displaystat

}  // namespace bipe
