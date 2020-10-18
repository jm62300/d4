#include "WrapperSolver.hpp"
#include "WrapperMinisat.hpp"

namespace d4
{
/**
   Wrapper to get a solver able to solve the input problem for the
   compilation/counting problems.

   @param[in] vm, the options.
 */
WrapperSolver *WrapperSolver::makeWrapperSolver(po::variables_map &vm)
{
  std::string s = vm["solver"].as<std::string>();

  if(s == "minisat") return new WrapperMinisat();
  
  return NULL;
} // makeWrapperSolver


/**
   Wrapper to get a solver able to solve the input problem for the preprocessing
   step.

   @param[in] vm, the options.
 */
WrapperSolver *WrapperSolver::makeWrapperSolverPreproc(po::variables_map &vm)
{
  std::string s = vm["preproc-solver"].as<std::string>();

  if(s == "minisat") return new WrapperMinisat();
  
  return NULL;
} // makeWrapperSolverPreproc


}
