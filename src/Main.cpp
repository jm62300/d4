#include <iostream>
#include <cassert>
#include <vector>
#include <boost/program_options.hpp>

#include "problem/ProblemManager.hpp"

/**
   The main function!
*/
int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::options_description desc{"Options"};
  desc.add_options()
#include "option.dsc"
      ;

  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);

  try
  {    
    po::notify(vm);
  }
  catch (const po::error &ex)
  {
    std::cerr << ex.what() << '\n';
    exit(1);
  }

  // help or problem with the command line
  if (vm.count("help") || !vm.count("input"))
  {
    if(!vm.count("help")) std::cout << "Some parameters are missing, please read the README\n";
    std::cout << "USAGE: " << argv[0] << " -i INPUT -m METH [OPTIONS]\n";
    std::cout << desc << '\n';
    exit(!vm.count("help"));
  }
  
  d4::ProblemManager *p = d4::ProblemManager::makeProblemManager(vm);
  p->display(std::cout);
  
  return 0;
}// main
