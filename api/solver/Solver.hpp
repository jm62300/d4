#pragma once

#include <memory>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/problem/ProblemTypes.hpp"
#include "api/result/SolverResult.hpp"

namespace parser {
struct Formula;
}

namespace d4::api {

enum class WeightType { INT, FLOAT, COMPLEX };

enum class GateType { AND, OR, XOR, ATMOST, IDENTITY, CLAUSE };

struct Gate {
  GateType gateType;
  std::vector<int> inputs;
  int output = 0;
  int threshold = 0;
};

/**
 * @brief Class for solving and compiling a formula using configured options, designed for the API and wrappers.
 */
class Solver {
 public:
  /**
   * @brief Construct a Solver with logic gates representing a Boolean Circuit.
   *
   * @param gates       The gates of the Boolean Circuit.
   * @param nbVars      The number of variables (if 0, it will be auto-detected from gates).
   * @param config      The DPLL method configuration (defaults to default settings).
   */
  Solver(const std::vector<Gate>& gates, int nbVars = 0,
         const d4::OptionDpllStyleMethod& config = d4::OptionDpllStyleMethod());
  /**
   * @brief Construct a Solver with clauses and number of variables.
   *
   * @param clauses     The clauses of the CNF formula.
   * @param nbVars      The number of variables (if 0, it will be auto-detected from clauses).
   * @param config      The DPLL method configuration (defaults to default settings).
   */
  Solver(const std::vector<std::vector<int>>& clauses, int nbVars = 0,
         const d4::OptionDpllStyleMethod& config = d4::OptionDpllStyleMethod());

  /**
   * @brief Construct a Solver from a DIMACS (.cnf) or circuit (.bc) file.
   *
   * @param filepath    The path to the DIMACS or circuit file.
   * @param config      The DPLL method configuration (defaults to default settings).
   */
  Solver(const std::string& filepath,
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
   * @brief Set weights for literals to perform Weighted Model Counting.
   *
   * @param weights     A map of DIMACS literal integers (e.g. 1, -1) to their weight string representations.
   * @param type        The weight type (INT, FLOAT, or COMPLEX, default FLOAT).
   */
  void setWeights(const std::map<int, std::string>& weights, WeightType type = WeightType::FLOAT);

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

  /**
   * @brief Destructor.
   */
  ~Solver();

 private:
  std::vector<d4::BcGate> buildGates() const;

  d4::OptionDpllStyleMethod options_;
  std::unique_ptr<parser::Formula> formula_;
  bool refinement_ = true;
};

}  // namespace d4::api
