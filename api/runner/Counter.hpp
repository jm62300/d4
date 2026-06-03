#pragma once

#include <memory>
#include <iostream>
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "c++/parser/ParserDimacs.hpp"
#include "api/result/CounterResult.hpp"

namespace d4::api {

/**
 * @brief Class for counting models of a formula using configured options, designed for the API and Python wrapper.
 */
class Counter {
 public:
  /**
   * @brief Construct a Counter with a pre-parsed formula.
   *
   * @param formula     The parsed input formula.
   * @param config      The DPLL method configuration (defaults to default settings).
   */
  Counter(const parser::Formula& formula,
          const d4::OptionDpllStyleMethod& config = d4::OptionDpllStyleMethod());

  /**
   * @brief Get the configuration options (const version).
   * @return The DPLL method configuration option.
   */
  const d4::OptionDpllStyleMethod& getOptions() const;

  /**
   * @brief Get the configuration options (non-const version).
   * @return The DPLL method configuration option.
   */
  d4::OptionDpllStyleMethod& getOptions();

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
   * @brief Executes the model counting algorithm and returns a CounterResult object.
   *
   * @param out         The output stream to print standard logs/results to (defaults to std::cout).
   * @return std::unique_ptr<CounterResult> The model counting result and solver statistics.
   */
  std::unique_ptr<CounterResult> run(std::ostream& out = std::cout);

 private:
  d4::OptionDpllStyleMethod options_;
  parser::Formula formula_;
};

}  // namespace d4::api
