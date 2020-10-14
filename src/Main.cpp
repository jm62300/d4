#include <iostream>
#include <cassert>

#include <boost/program_options.hpp>

void on_age(int age)
{
  std::cout << "On age: " << age << '\n';
}


/**
   The main function!
*/
int main(int argc, char** argv)
{
  try
  {
    namespace po = boost::program_options;
    po::options_description desc{"Options"};
    desc.add_options()
#include "option.dsc"
        ;

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) std::cout << desc << '\n';    
    std::cout << "input file: " << vm["input"].as<std::string>() << '\n';
  }
  catch (const boost::program_options::error &ex)
  {
    std::cerr << ex.what() << '\n';
  }

  return 0;
}// main
