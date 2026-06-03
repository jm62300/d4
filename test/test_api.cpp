#include <iostream>
#include <memory>
#include <sstream>
#include <cassert>
#include "c++/parser/ParserDimacs.hpp"
#include "api/solver/Solver.hpp"
#include "api/result/SolverResult.hpp"

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

  std::cout << "Creating Solver..." << std::endl;
  d4::api::Solver solver(formula);

  std::cout << "Running direct model counting..." << std::endl;
  std::stringstream countLog;
  std::unique_ptr<d4::api::CountResult> countResult = solver.count(countLog);

  std::cout << "--- Direct WMC Results ---" << std::endl;
  std::cout << "Model count: " << countResult->getResult() << std::endl;
  std::cout << "Recursive calls: " << countResult->getNbRecursiveCalls() << std::endl;
  std::cout << "Formula splits: " << countResult->getNbSplits() << std::endl;
  std::cout << "Decision nodes: " << countResult->getNbDecisionNodes() << std::endl;
  std::cout << "Solve time: " << countResult->getSolveTime() << " seconds" << std::endl;

  assert(countResult->getResult() == "1");

  // Verify typed C++ accessors
  std::cout << "Testing C++ typed result accessors..." << std::endl;
  assert(countResult->getIntResult() == 1);
  try {
    countResult->getFloatResult();
    assert(false); // Should throw exception
  } catch (const std::runtime_error& e) {
    std::cout << "Expected exception caught: " << e.what() << std::endl;
  }

  std::cout << "\nRunning compilation to d-DNNF..." << std::endl;
  std::stringstream compileLog;
  std::unique_ptr<d4::api::CompileResult> compileResult = solver.compile(compileLog);

  std::cout << "--- Compilation Results ---" << std::endl;
  std::cout << "d-DNNF Nodes: " << compileResult->getNbNodes() << std::endl;
  std::cout << "d-DNNF Edges: " << compileResult->getNbEdges() << std::endl;
  std::cout << "Recursive calls: " << compileResult->getNbRecursiveCalls() << std::endl;
  std::cout << "Solve time: " << compileResult->getSolveTime() << " seconds" << std::endl;

  std::cout << "\n--- Compiled d-DNNF Circuit ---" << std::endl;
  std::string nnfStr = compileResult->getNNFString();
  std::cout << nnfStr << std::endl;

  std::cout << "--- Querying Compiled Circuit ---" << std::endl;
  // Query 1: Empty assumptions (should equal the total model count)
  std::vector<int> queryEmpty = {};
  std::cout << "Query empty assumptions count: " << compileResult->count(queryEmpty) << " (Expected: 1)" << std::endl;
  assert(compileResult->count(queryEmpty) == "1");
  assert(compileResult->isSAT(queryEmpty) == true);

  // Query 2: Valid assumption {1}
  std::vector<int> queryVal1 = {1};
  std::cout << "Query {1} count: " << compileResult->count(queryVal1) << " (Expected: 1)" << std::endl;
  std::cout << "Query {1} SAT: " << (compileResult->isSAT(queryVal1) ? "SAT" : "UNSAT") << " (Expected: SAT)" << std::endl;
  assert(compileResult->count(queryVal1) == "1");
  assert(compileResult->isSAT(queryVal1) == true);

  // Query 3: Conflicting assumption {1, 2} (since 2 must be false)
  std::vector<int> queryConf = {1, 2};
  std::cout << "Query {1, 2} count: " << compileResult->count(queryConf) << " (Expected: 0)" << std::endl;
  std::cout << "Query {1, 2} SAT: " << (compileResult->isSAT(queryConf) ? "SAT" : "UNSAT") << " (Expected: UNSAT)" << std::endl;
  assert(compileResult->count(queryConf) == "0");
  assert(compileResult->isSAT(queryConf) == false);

  std::cout << "\nAll API tests passed successfully!" << std::endl;
  return 0;
}
