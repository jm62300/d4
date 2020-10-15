#ifndef d4_problem_ProblemManagerCnf_hpp
#define d4_problem_ProblemManagerCnf_hpp

#include <boost/program_options.hpp>

#include "ProblemManager.hpp"

namespace d4
{
namespace po = boost::program_options;

class ProblemManagerCnf : public ProblemManager
{
 private:
  int nbVars;
  std::vector< std::vector<int> > clauses;
  
 public:
  ProblemManagerCnf(po::variables_map &vm);
  ~ProblemManagerCnf();

  int getNbVar(){return nbVars;}
  void display(std::ostream &out);
};
  
} // d4

#endif
