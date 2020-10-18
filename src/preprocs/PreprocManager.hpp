#ifndef d4_src_preprocs_PreprocManager_hpp
#define d4_src_preprocs_PreprocManager_hpp

#include <vector>
#include <boost/program_options.hpp>

#include "../problem/ProblemTypes.hpp"
#include "../problem/ProblemManager.hpp"

namespace d4
{
namespace po = boost::program_options;
class PreprocManager
{
 private:
 public:
  static PreprocManager *makePreprocManager(po::variables_map &vm);

  virtual ~PreprocManager(){}

  /* The preprocessing is directly applied on p */
  virtual void run(ProblemManager &p) = 0;
};
}

#endif
