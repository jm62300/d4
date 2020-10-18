#ifndef d4_problem_ProblemManagerCnf_hpp
#define d4_problem_ProblemManagerCnf_hpp

#include <boost/program_options.hpp>

#include "../ProblemTypes.hpp"
#include "../ProblemManager.hpp"
#include "../../solvers/WrapperSolver.hpp"

namespace d4
{
namespace po = boost::program_options;

class ProblemManagerCnf : public ProblemManager
{
 private:
  int nbVars;
  std::vector< std::vector<Lit> > clauses;
  WrapperSolver *ws;
  
 public:
  ProblemManagerCnf(po::variables_map &vm);
  ~ProblemManagerCnf();

  int getNbVar(){return nbVars;}
  void display(std::ostream &out);

  std::vector< std::vector<Lit> > &getClauses(){return clauses;}
};
  
} // d4

#endif
