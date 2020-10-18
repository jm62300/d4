#ifndef d4_src_preprocs_PreprocBasic_hpp
#define d4_src_preprocs_PreprocBasic_hpp

#include <vector>
#include <boost/program_options.hpp>

#include "../problem/ProblemTypes.hpp"
#include "../solvers/WrapperSolver.hpp"
#include "PreprocManager.hpp"

namespace d4
{
namespace po = boost::program_options;
class PreprocBasic : public PreprocManager
{
 private:
  WrapperSolver *ws;
  
 public:
  PreprocBasic(po::variables_map &vm);
  void run(ProblemManager &p);
};

} // d4

#endif
