#include "ProblemManagerCnf.hpp"
#include "ParserDimacs.hpp"

namespace d4
{
/**
   Constructor. Take as arguments on the options.

   @param[in] vm, the arguments on the command line.
 */
ProblemManagerCnf::ProblemManagerCnf(po::variables_map &vm)
{
  ParserDimacs parser;
  nbVars = parser.parse_DIMACS(vm["input"].as<std::string>(), clauses);
} // constructor


/**
   Destructor.
 */
ProblemManagerCnf::~ProblemManagerCnf()
{
  clauses.clear();
  nbVars = 0;
} // destructor


/**
   Display the problem.

   @param[out] out, the stream where the messages are redirected.
 */
void ProblemManagerCnf::display(std::ostream &out)
{
  out << "p cnf " << nbVars << " " << clauses.size() << "\n";
  for(auto cl : clauses)
  {
    for(auto l : cl) out << l << " ";
    out << "0\n";
  }
} // diplay

}
