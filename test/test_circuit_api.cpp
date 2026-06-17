#include <iostream>
#include <memory>
#include <sstream>
#include <fstream>
#include <cassert>
#include "c++/parser/ParserCircuit.hpp"
#include "api/solver/Solver.hpp"
#include "api/result/SolverResult.hpp"

int main() {
  std::string inputPath = "instancesTest/circuits/test.bc";
  std::cout << "Reading circuit file: " << inputPath << std::endl;

  parser::Formula formula;
  parser::ParserCircuit parserCircuit;
  try {
    parserCircuit.parse_circuit(inputPath, formula);
    std::cout << formula << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse circuit: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Creating Solver for circuit..." << std::endl;
  d4::api::Solver solver(formula);

  std::cout << "Running direct model counting on circuit..." << std::endl;
  std::stringstream countLog;
  std::unique_ptr<d4::api::CountResult> countResult = solver.count(countLog);

  std::cout << "--- Direct WMC Results ---" << std::endl;
  std::cout << "Model count: " << countResult->getResult() << std::endl;
  std::cout << "Recursive calls: " << countResult->getNbRecursiveCalls() << std::endl;
  std::cout << "Formula splits: " << countResult->getNbSplits() << std::endl;
  std::cout << "Decision nodes: " << countResult->getNbDecisionNodes() << std::endl;
  std::cout << "Solve time: " << countResult->getSolveTime() << " seconds" << std::endl;

  assert(countResult->getResult() == "15");
  assert(countResult->getIntResult() == 15);

  std::cout << "\nRunning compilation of circuit to d-DNNF..." << std::endl;
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
  std::vector<int> queryEmpty = {};
  std::cout << "Query empty assumptions count: " << compileResult->count(queryEmpty) << " (Expected: 15)" << std::endl;
  assert(compileResult->count(queryEmpty) == "15");
  assert(compileResult->isSAT(queryEmpty) == true);

  std::cout << "\nAll Circuit API tests passed successfully!" << std::endl;
  return 0;
}
