#pragma once

#include <memory>
#include <iostream>
#include <vector>
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "c++/parser/ParserDimacs.hpp"
#include "api/result/SolverResult.hpp"

namespace d4::api {

/**
 * @brief Class for solving and compiling a formula using configured options, designed for the API and wrappers.
 */
class Solver {
 public:
  /**
   * @brief Construct a Solver with a pre-parsed formula.
   *
   * @param formula     The parsed input formula.
   * @param config      The DPLL method configuration (defaults to default settings).
   */
  Solver(const parser::Formula& formula,
         const d4::OptionDpllStyleMethod& config = d4::OptionDpllStyleMethod());

  /**
   * @brief Get the configuration options (const version).
   * @return The DPLL method configuration option.
   */
  const d4::OptionDpllStyleMethod& getOptions() const;

  /**
   * @brief Set the configuration options.
   * @param config The DPLL method configuration option.
   */
  void setOptions(const d4::OptionDpllStyleMethod& config);

  /**
   * @brief Get the parsed formula.
   * @return The parsed input formula.
   */
  const parser::Formula& getFormula() const;

  /**
   * @brief Set the parsed formula.
   * @param formula The parsed input formula.
   */
  void setFormula(const parser::Formula& formula);

  /**
   * @brief Set weights for literals to perform Weighted Model Counting.
   *
   * @param weights     A map of DIMACS literal integers (e.g. 1, -1) to their weight string representations.
   * @param type        The weight type (INT, FLOAT, or COMPLEX, default FLOAT).
   */
  void setWeights(const std::map<int, std::string>& weights, parser::WeightType type = parser::WeightType::FLOAT);

  /**
   * @brief Set the variables to project on for Projected Model Counting.
   *
   * @param projectionVars The variables (1-based DIMACS indices) to project on.
   */
  void setProjectionVariables(const std::vector<int>& projectionVars);

  /**
   * @brief Set refinement option for Projected Model Counting.
   */
  void setRefinement(bool refinement);

  /**
   * @brief Get refinement option for Projected Model Counting.
   */
  bool getRefinement() const;

  /**
   * @brief Executes the model counting algorithm and returns a CountResult object.
   *
   * @param out         The output stream to print standard logs/results to (defaults to std::cout).
   * @return std::unique_ptr<CountResult> The model counting result and solver statistics.
   */
  std::unique_ptr<CountResult> count(std::ostream& out = std::cout);

  /**
   * @brief Compiles the formula to a d-DNNF circuit and returns a CompileResult object.
   *
   * @param out         The output stream to print standard logs/results to (defaults to std::cout).
   * @return std::unique_ptr<CompileResult> The compilation result holding the d-DNNF circuit.
   */
  std::unique_ptr<CompileResult> compile(std::ostream& out = std::cout);

 private:
  std::vector<d4::BcGate> buildGates() const;

  d4::OptionDpllStyleMethod options_;
  parser::Formula formula_;
  bool refinement_ = true;
};

}  // namespace d4::api
