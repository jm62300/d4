#include "WrapperSolver.hpp"
#include "WrapperMinisat.hpp"

namespace d4
{
WrapperSolver *WrapperSolver::makeWrapperSolver(po::variables_map &vm)
{
  std::string in = vm["solver"].as<std::string>();

  if(in == "minisat") return new WrapperMinisat();
  
  return NULL;
} // makeWrapperSolver

}
