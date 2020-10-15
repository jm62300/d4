#ifndef d4_problem_ProblemManager_hpp
#define d4_problem_ProblemManager_hpp

#include <boost/program_options.hpp>

namespace d4
{
namespace po = boost::program_options;
class ProblemManager
{
 private:
 public:
  static ProblemManager *makeProblemManager(po::variables_map &vm);
  
  virtual ~ProblemManager(){;}
  virtual int getNbVar() = 0;
  virtual void display(std::ostream &out) = 0;
};
}

#endif
