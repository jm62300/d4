#include <iostream>
#include <cassert>
#include <vector> 

#include <boost/program_options.hpp>

#include "circuit/Circuit.hpp"
#include "parsing/ParserDimacs.hpp"


#include "circuit/ClauseGate.hpp"
#include "circuit/AndGate.hpp"

void on_age(int age)
{
  std::cout << "On age: " << age << '\n';
}


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
    catch (const boost::program_options::error &ex)
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

    
    std::cout << "input file: " << vm["input"].as<std::string>() << '\n';


    int tab1[] = {1,2,3}, tab2[] = {4,2,3};
    std::vector<int> v1(std::begin(tab1), std::end(tab1));
    std::vector<int> v2(std::begin(tab2), std::end(tab2));

    std::vector<d4::Circuit *> av;
    av.push_back(new d4::ClauseGate(v1));
    av.push_back(new d4::ClauseGate(v2));
    d4::Circuit *ac = new d4::AndGate(av);

    std::cout << (*ac) << "\n";
    
  return 0;
}// main
