#ifndef d4_src_solvers_WrapperSolver_hpp
#define d4_src_solvers_WrapperSolver_hpp

#include "../problem/ProblemTypes.hpp"
#include "../problem/ProblemManager.hpp"
#include <boost/program_options.hpp>

namespace d4
{
namespace po = boost::program_options;
class WrapperSolver
{
  private:

 public:
  static WrapperSolver *makeWrapperSolver(po::variables_map &vm);

  virtual void initSolver(ProblemManager &p) = 0;
};
} // d4

#endif
