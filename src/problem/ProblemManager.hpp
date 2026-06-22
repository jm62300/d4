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

#include <map>

#include "src/exceptions/FactoryException.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "src/utils/MpzTypes.hpp"

namespace d4 {
namespace mpz = d4MpzTypes;

enum ProblemInputType { PB_CNF, PB_TCNF, PB_CIRC, PB_QBF, PB_NONE };

class ProblemInputTypeManager {
 public:
  /**
   * @brief This function maps the enum type option to the associate string
   * option.
   *
   * @test getInputType("cnf") == PB_CNF
   *
   * @param m is the enum type option.
   * @return the string associate with this type.
   */
  static std::string getInputType(const ProblemInputType& m) {
    if (m == PB_CNF) return "cnf";
    if (m == PB_CIRC) return "circuit";
    if (m == PB_TCNF) return "cnf+theory";
    if (m == PB_QBF) return "qbf";

    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType

  /**
   * @brief Display the possible options on the given stream.
   *
   * @param out is the output stream where write the information.
   */
  static void displayPossibleOptions(std::ostream& out) {
    out << "\033[1m\033[31mProblem types handling by d4: \033[0m\n";
    out << "\t- CNF formula following the Dimacs format -> cnf\n";
    out << "\t- Circuit formula -> circuit\n";
    out << "\t- CNF formula together with a CNF theory following the Dimacs "
           "format -> tcnf\n";
    out << "\t- QBF formula following the QDimacs format -> qcnf\n";
  }  // displayPossibleOptions

  /***
   * @brief This function maps the string option to the enum type.
   *
   * @param[in] m is the string representing the option.
   */
  static ProblemInputType getInputType(const std::string& m) {
    if (m == "cnf") return PB_CNF;
    if (m == "circuit") return PB_CIRC;
    if (m == "tcnf") return PB_TCNF;
    if (m == "qbf") return PB_QBF;

    displayPossibleOptions(std::cerr);
    throw(FactoryException("Operator Type unknown", __FILE__, __LINE__));
  }  // getOperatorType
};

enum ProblemTranslateType { TRANSLATE_CNF, TRANSLATE_PCNF, TRANSLATE_NONE };

class ProblemTranslateTypeManager {
 public:
  /**
   * @brief This function maps the enum type option to the associate string
   * option.
   *
   * @test getInputType("cnf") == PB_CNF
   *
   * @param m is the enum type option.
   * @return the string associate with this type.
   */
  static std::string getInputType(const ProblemTranslateType& m) {
    if (m == TRANSLATE_CNF) return "cnf";
    if (m == TRANSLATE_PCNF) return "pcnf";
    if (m == TRANSLATE_NONE) return "none";
    throw(FactoryException("Type unknown", __FILE__, __LINE__));
  }  // getOutputType

  /**
   * @brief Display the possible options on the given stream.
   *
   * @param out is the output stream where write the information.
   */
  static void displayPossibleOptions(std::ostream& out) {
    out << "\033[1m\033[31mTranslation type handling by d4: \033[0m\n";
    out << "\t- Do not translate the formula -> none\n";
    out << "\t- Translate the formula into a CNF formula -> cnf\n";
    out << "\t- Translate the formula into a projected CNF formula -> pcnf\n";
  }  // displayPossibleOptions

  /***
   * @brief This function maps the string option to the enum type.
   *
   * @param[in] m is the string representing the option.
   */
  static ProblemTranslateType getInputType(const std::string& m) {
    if (m == "cnf") return TRANSLATE_CNF;
    if (m == "pcnf") return TRANSLATE_PCNF;
    if (m == "none") return TRANSLATE_NONE;

    displayPossibleOptions(std::cerr);
    throw(FactoryException("Translation type unknown", __FILE__, __LINE__));
  }  // getOperatorType
};

class ProblemManager {
 protected:
  unsigned m_nbVar = 0;
  std::vector<std::vector<Var>> m_quantification;
  std::vector<unsigned> m_order;
  ProblemInputType m_problemInputType;
  std::vector<BcGate> m_gates;
  std::map<Lit, std::string> m_weightMap;
  bool m_isUnsat = false;

 public:
  /**
   * @brief Default constructor.
   * Automatically calls the default constructor of Quantification.
   */
  ProblemManager() = default;

  /**
   * @brief Constructs the ProblemManager and initializes the SAT problem
   * structures.
   *
   * @param type            The string identifier for the problem type (e.g.,
   * "cnf").
   * @param nbVar           The total number of variables in the problem.
   * @param quantifications 2D vector representing the quantified variable
   * blocks.
   * @param weightMap       Map associating integer literals to their
   * string-represented weights.
   * @param gates           2D vector of raw integers representing logical
   * gates/clauses.
   * @param out             Output stream for logging (Note: currently unused in
   * this scope).
   */
  ProblemManager(const std::string& type, unsigned nbVar,
                 const std::vector<std::vector<Var>>& quantifications,
                 const std::map<Lit, std::string>& weightMap,
                 const std::vector<BcGate>& gates, std::ostream& out)
      : m_nbVar(nbVar),
        m_quantification(quantifications),
        m_problemInputType(ProblemInputTypeManager::getInputType(type)),
        m_gates(gates),
        m_weightMap(weightMap) {}

 public:
  /**
   * @brief Gets the weight map associating literals to their string weights.
   *
   * @return A constant reference to the weight map to prevent unnecessary
   * copying.
   */
  const std::map<Lit, std::string>& getWeightMap() const { return m_weightMap; }

  /**
   * @brief Gets the quantified variable blocks.
   *
   * @return A constant reference to the 2D vector of quantifications.
   */
  const std::vector<std::vector<Var>>& getQuantification() const {
    return m_quantification;
  }

  /**
   * @brief Gets the variable ordering sequence.
   *
   * @return A constant reference to the vector containing the order of
   * variables.
   */
  const std::vector<unsigned>& getOrder() const { return m_order; }

  /**
   * @brief Gets the designated problem input type (e.g., CNF, wCNF).
   *
   * @return The ProblemInputType enum value. Returned by value as it is
   * lightweight.
   */
  ProblemInputType getProblemInputType() const { return m_problemInputType; }

  /**
   * @brief Gets the logical gates / clauses of the problem.
   *
   * @return A constant reference to the 2D vector of literals representing the
   * gates.
   */
  const std::vector<BcGate>& getGates() const { return m_gates; }

  /**
   * @brief Gets the total number of variables in the problem.
   *
   * @return The number of variables (unsigned integer). Returned by value.
   */
  unsigned getNbVar() const { return m_nbVar; }

  /**
   * @brief Checks if the problem has already been determined to be
   * unsatisfiable.
   *
   * @return True if the problem is definitively UNSAT, false otherwise.
   * Returned by value.
   */
  bool isUnsat() const { return m_isUnsat; }

  /**
   * @brief Print all attributes of this object to the given stream.
   *
   * @param[out] out is the stream where the messages are redirected.
   */
  void display(std::ostream& out) const {
    out << "nbVar: " << m_nbVar << "\n";
    out << "isUnsat: " << m_isUnsat << "\n";
    out << "problemInputType: "
        << ProblemInputTypeManager::getInputType(m_problemInputType) << "\n";

    out << "order:";
    for (auto v : m_order) out << " " << v;
    out << "\n";

    out << "quantification (" << m_quantification.size() << " block(s)):\n";
    for (std::size_t i = 0; i < m_quantification.size(); ++i) {
      out << "  [" << i << "]:";
      for (auto v : m_quantification[i]) out << " " << v;
      out << "\n";
    }

    out << "weightMap (" << m_weightMap.size() << " entries):\n";
    for (const auto& [lit, w] : m_weightMap)
      out << "  " << lit.human() << " -> " << w << "\n";

    out << "gates (" << m_gates.size() << "):\n";
    for (auto g : m_gates) {
      out << "  ";
      g.display(out);
      out << "\n";
    }
  }  // display

  /**
   * @brief Translate the circuit into a set of clauses using the tseitin
   * encoding.
   *
   * @param[out] tseitinGates is the set of clauses.
   */
  void toCnf(std::vector<BcGate>& tseitinGates) const {
    for (const auto& g : getGates()) {
      switch (g.gateType) {
        case BcGateType::AND: {
          std::vector<Lit> fwd = {g.output};
          for (auto l : g.input) {
            tseitinGates.push_back(
                {{l, ~g.output}, lit_Undef, BcGateType::CLAUSE});
            fwd.push_back(~l);
          }
          tseitinGates.push_back({fwd, lit_Undef, BcGateType::CLAUSE});
          break;
        }
        case BcGateType::OR: {
          std::vector<Lit> fwd = {~g.output};
          for (auto l : g.input) {
            tseitinGates.push_back(
                {{~l, g.output}, lit_Undef, BcGateType::CLAUSE});
            fwd.push_back(l);
          }
          tseitinGates.push_back({fwd, lit_Undef, BcGateType::CLAUSE});
          break;
        }
        case BcGateType::IDENTITY:
          tseitinGates.push_back(
              {{g.input[0], ~g.output}, lit_Undef, BcGateType::CLAUSE});
          tseitinGates.push_back(
              {{~g.input[0], g.output}, lit_Undef, BcGateType::CLAUSE});
          break;
        case BcGateType::CLAUSE:
          tseitinGates.push_back(g);
          break;
        default:
          break;
      }
    }
  }  // toCnf
};
}  // namespace d4
