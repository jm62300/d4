#include <iostream>
#include <memory>
#include <sstream>
#include <fstream>
#include <cassert>
#include <cmath>
#include "api/solver/Solver.hpp"
#include "api/result/SolverResult.hpp"

int main() {
  std::cout << "Creating a simple 3-variable CNF formula..." << std::endl;
  // Formula: (x1 v x2) ^ (~x2 v x3)
  std::vector<std::vector<int>> clauses = {{1, 2}, {-2, 3}};

  std::cout << "Creating Solver and configuring projection variables programmatically..." << std::endl;
  d4::api::Solver solver(clauses, 3);

  // Set projection variables to {1, 3}
  solver.setProjectionVariables({1, 3});

  // Verify and test refinement option getters and setters
  assert(solver.getRefinement() == true);
  solver.setRefinement(false);
  assert(solver.getRefinement() == false);
  solver.setRefinement(true);
  assert(solver.getRefinement() == true);

  std::cout << "Running direct Projected Model Counting (ProjMC)..." << std::endl;
  std::stringstream countLog;
  std::unique_ptr<d4::api::CountResult> countResult = solver.count(countLog);

  unsigned long directCount = countResult->getIntResult().get_ui();
  std::cout << "Direct ProjMC Result: " << directCount << " (Expected: 3)" << std::endl;
  assert(directCount == 3);

  // Verify recursive calls and splits are populated/reasonable
  std::cout << "ProjMC Stats: recursive calls=" << countResult->getNbRecursiveCalls()
            << ", splits=" << countResult->getNbSplits()
            << ", solve time=" << countResult->getSolveTime() << std::endl;

  std::cout << "\nRunning compilation with projection to d-DNNF..." << std::endl;
  std::stringstream compileLog;
  std::unique_ptr<d4::api::CompileResult> compileResult = solver.compile(compileLog);

  std::cout << "--- Compilation Results ---" << std::endl;
  std::cout << "d-DNNF Nodes: " << compileResult->getNbNodes() << std::endl;
  std::cout << "d-DNNF Edges: " << compileResult->getNbEdges() << std::endl;

  std::cout << "\n--- Compiled d-DNNF Circuit ---" << std::endl;
  std::string nnfStr = compileResult->getNNFString();
  std::cout << nnfStr << std::endl;

  std::cout << "--- Querying Compiled Circuit (Projected) ---" << std::endl;
  
  // Query 1: Empty assumptions (should equal the total projected count of 3)
  std::vector<int> queryEmpty = {};
  unsigned long countEmpty = std::stoul(compileResult->count(queryEmpty));
  std::cout << "Projected count with empty assumptions: " << countEmpty << " (Expected: 3)" << std::endl;
  assert(countEmpty == 3);
  assert(compileResult->isSAT(queryEmpty) == true);

  // Query 2: Assumption {1} (satisfying projected assignments: {1, 3} and {1, -3} -> 2)
  std::vector<int> query1 = {1};
  unsigned long count1 = std::stoul(compileResult->count(query1));
  std::cout << "Projected count with assumption {1}: " << count1 << " (Expected: 2)" << std::endl;
  assert(count1 == 2);
  assert(compileResult->isSAT(query1) == true);

  // Query 3: Assumption {-1} (satisfying projected assignment: {-1, 3} -> 1)
  std::vector<int> queryNot1 = {-1};
  unsigned long countNot1 = std::stoul(compileResult->count(queryNot1));
  std::cout << "Projected count with assumption {-1}: " << countNot1 << " (Expected: 1)" << std::endl;
  assert(countNot1 == 1);
  assert(compileResult->isSAT(queryNot1) == true);

  // Query 4: Assumption {-3} (satisfying projected assignment: {1, -3} -> 1)
  std::vector<int> queryNot3 = {-3};
  unsigned long countNot3 = std::stoul(compileResult->count(queryNot3));
  std::cout << "Projected count with assumption {-3}: " << countNot3 << " (Expected: 1)" << std::endl;
  assert(countNot3 == 1);
  assert(compileResult->isSAT(queryNot3) == true);

  // Query 5: Conflicting assumption {-1, -3} (no satisfying projected assignment -> 0)
  std::vector<int> queryConf = {-1, -3};
  unsigned long countConf = std::stoul(compileResult->count(queryConf));
  std::cout << "Projected count with conflicting assumptions {-1, -3}: " << countConf << " (Expected: 0)" << std::endl;
  assert(countConf == 0);
  assert(compileResult->isSAT(queryConf) == false);

  std::cout << "\nAll ProjMC API tests passed successfully!" << std::endl;
  return 0;
}
