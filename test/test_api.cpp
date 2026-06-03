#include <iostream>
#include <memory>
#include "c++/parser/ParserDimacs.hpp"
#include "api/runner/Counter.hpp"
#include "api/result/CounterResult.hpp"

int main() {
  std::string inputPath = "instancesTest/cnfs/smallSAT.cnf";

  parser::Formula formula;
  parser::ParserDimacs parserDimacs;
  try {
    parserDimacs.parse_DIMACS(inputPath, formula);
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse formula: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Creating Counter..." << std::endl;
  d4::api::Counter counter(formula);

  std::cout << "Running model counting..." << std::endl;
  // Redirect output to a stringstream to avoid polluting stdout with solver internal logs during the test
  std::stringstream solverLog;
  std::unique_ptr<d4::api::CounterResult> result = counter.run(solverLog);

  std::cout << "--- RESULTS ---" << std::endl;
  std::cout << "Model count: " << result->getResult() << std::endl;
  std::cout << "Recursive calls: " << result->getNbRecursiveCalls() << std::endl;
  std::cout << "Formula splits: " << result->getNbSplits() << std::endl;
  std::cout << "Decision nodes: " << result->getNbDecisionNodes() << std::endl;
  std::cout << "Solve time: " << result->getSolveTime() << " seconds" << std::endl;

  return 0;
}
