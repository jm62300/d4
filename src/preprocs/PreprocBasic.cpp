#include "../problem/cnf/ProblemManagerCnf.hpp"
#include "PreprocBasic.hpp"

namespace d4
{

/**
   The constructor. 

   @param[in] vm, the options used (solver).
 */
PreprocBasic::PreprocBasic(po::variables_map &vm)
{
  ws = WrapperSolver::makeWrapperSolverPreproc(vm);
} // constructor


/**
   Destructor.
 */
PreprocBasic::~PreprocBasic()
{
  delete ws;
} // destructor

/**
   The preprocessing itself.

   @param[out] p, the problem we want to preprocess.
 */
void PreprocBasic::run(ProblemManager &p)
{  
  ws->initSolver(p);

  try
  {
    ProblemManagerCnf &pcnf = dynamic_cast<ProblemManagerCnf&>(p);      
    
    if(!ws->solve()) // p is UNSAT 
    {
      std::vector<std::vector<Lit> > &ret = pcnf.getClauses();
      ret.clear();
    
      std::vector<Lit> cl;
      Lit l = Lit(1, false);
    
      cl.push_back(l);
      ret.push_back(cl);
    
      cl[0] = l.neg();
      ret.push_back(cl);
    }
    else // SAT: extract the clauses from the solver.
    {
      ws->getSimplifiedFormula(p);
    }
  }
  catch (std::bad_cast& bc)
  {
    std::cerr << "bad_cast caught: " << bc.what() << '\n';
    std::cerr << "A CNF formula was expeted\n";
  }
} // run

}
