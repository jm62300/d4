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
      boost::program_options::options_description desc{"Options"};
      desc.add_options()
        ("help,h", "Help screen")
        ("pi", boost::program_options::value<float>()->default_value(3.14f), "Pi")
        ("age", boost::program_options::value<int>()->notifier(on_age), "Age");

      boost::program_options::variables_map vm;
      boost::program_options::store(parse_command_line(argc, argv, desc), vm);
      boost::program_options::notify(vm);

      if (vm.count("help")) std::cout << desc << '\n';
      else if (vm.count("age")) std::cout << "Age: " << vm["age"].as<int>() << '\n';
      else if (vm.count("pi")) std::cout << "Pi: " << vm["pi"].as<float>() << '\n';
    }
  catch (const boost::program_options::error &ex)
    {
      std::cerr << ex.what() << '\n';
    }

  return 0;
}// main
