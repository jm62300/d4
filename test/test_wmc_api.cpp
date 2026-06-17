#include <iostream>
#include <memory>
#include <sstream>
#include <fstream>
#include <cassert>
#include <cmath>
#include "c++/parser/ParserDimacs.hpp"
#include "api/solver/Solver.hpp"
#include "api/result/SolverResult.hpp"

int main() {
  std::cout << "Creating a simple 2-variable CNF formula..." << std::endl;
  // Formula: (x1 v x2)
  std::string cnfData = 
      "p cnf 2 1\n"
      "1 2 0\n";

  parser::Formula formula;
  parser::ParserDimacs parserDimacs;
  try {
    parserDimacs.parse_DIMACS_from_data(cnfData, formula);
    std::cout << formula << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse formula: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Creating Solver and configuring literal weights programmatically..." << std::endl;
  d4::api::Solver solver(formula);

  // Configure weights:
  // w(1)  = 0.3, w(-1) = 0.7
  // w(2)  = 0.4, w(-2) = 0.6
  std::map<int, std::string> weights = {
    {1, "0.3"},
    {-1, "0.7"},
    {2, "0.4"},
    {-2, "0.6"}
  };
  solver.setWeights(weights, parser::WeightType::FLOAT);

  std::cout << "Running direct WMC..." << std::endl;
  std::stringstream countLog;
  std::unique_ptr<d4::api::CountResult> countResult = solver.count(countLog);

  double directCount = std::stod(countResult->getResult());
  std::cout << "Direct WMC Result: " << directCount << " (Expected: 0.58)" << std::endl;
  assert(std::abs(directCount - 0.58) < 1e-9);

  // Verify typed C++ accessors
  std::cout << "Testing C++ typed float result accessor..." << std::endl;
  boost::multiprecision::mpf_float floatVal = countResult->getFloatResult();
  double doubleVal = floatVal.convert_to<double>();
  assert(std::abs(doubleVal - 0.58) < 1e-9);

  std::cout << "\nRunning WMC compilation to d-DNNF..." << std::endl;
  std::stringstream compileLog;
  std::unique_ptr<d4::api::CompileResult> compileResult = solver.compile(compileLog);

  std::cout << "--- Compilation Results ---" << std::endl;
  std::cout << "d-DNNF Nodes: " << compileResult->getNbNodes() << std::endl;
  std::cout << "d-DNNF Edges: " << compileResult->getNbEdges() << std::endl;

  std::cout << "\n--- Compiled d-DNNF Circuit ---" << std::endl;
  std::string nnfStr = compileResult->getNNFString();
  std::cout << nnfStr << std::endl;

  std::cout << "--- Querying Compiled Circuit (Weighted) ---" << std::endl;
  
  // Query 1: Empty assumptions (should equal the total WMC count of 0.58)
  std::vector<int> queryEmpty = {};
  double wmcEmpty = std::stod(compileResult->count(queryEmpty));
  std::cout << "WMC with empty assumptions: " << wmcEmpty << " (Expected: 0.58)" << std::endl;
  assert(std::abs(wmcEmpty - 0.58) < 1e-9);
  assert(compileResult->isSAT(queryEmpty) == true);

  // Query 2: Assumption {1} (WMC satisfying assignments: {1, 2} and {1, -2} -> 0.12 + 0.18 = 0.3)
  std::vector<int> query1 = {1};
  double wmc1 = std::stod(compileResult->count(query1));
  std::cout << "WMC with assumption {1}: " << wmc1 << " (Expected: 0.30)" << std::endl;
  assert(std::abs(wmc1 - 0.30) < 1e-9);
  assert(compileResult->isSAT(query1) == true);

  // Query 3: Assumption {-1} (WMC satisfying assignment: {-1, 2} -> 0.28)
  std::vector<int> queryNot1 = {-1};
  double wmcNot1 = std::stod(compileResult->count(queryNot1));
  std::cout << "WMC with assumption {-1}: " << wmcNot1 << " (Expected: 0.28)" << std::endl;
  assert(std::abs(wmcNot1 - 0.28) < 1e-9);
  assert(compileResult->isSAT(queryNot1) == true);

  // Query 4: Conflicting assumption {-1, -2} (no satisfying assignment -> 0.0)
  std::vector<int> queryConf = {-1, -2};
  double wmcConf = std::stod(compileResult->count(queryConf));
  std::cout << "WMC with conflicting assumptions {-1, -2}: " << wmcConf << " (Expected: 0)" << std::endl;
  assert(std::abs(wmcConf - 0.0) < 1e-9);
  assert(compileResult->isSAT(queryConf) == false);

  std::cout << "\nAll WMC API tests passed successfully!" << std::endl;
  return 0;
}
