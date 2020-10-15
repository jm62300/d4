#include "ProblemManager.hpp"
#include "ProblemManagerCnf.hpp"

#include <iostream>


namespace d4
{

/**
   Select from the arguments store in vm the good problem manager and return it.

   @param[in] vm, the arguments on the command line.

   \return the problem manager that fits the command line.
 */
ProblemManager *ProblemManager::makeProblemManager(po::variables_map &vm)
{
  std::string in = vm["input"].as<std::string>();
  std::string extension = in.substr(in.find_last_of(".") + 1);

  if(extension == "cnf" || extension == "dimacs") return new ProblemManagerCnf(vm);
  
  return NULL;
}

}
