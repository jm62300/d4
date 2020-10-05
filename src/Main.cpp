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
#if 0
  try
    {
      options_description desc{"Options"};
      desc.add_options()
        ("help,h", "Help screen")
        ("pi", value<float>()->default_value(3.14f), "Pi")
        ("age", value<int>()->notifier(on_age), "Age");

      variables_map vm;
      store(parse_command_line(argc, argv, desc), vm);
      notify(vm);

      if (vm.count("help"))
        std::cout << desc << '\n';
      else if (vm.count("age"))
        std::cout << "Age: " << vm["age"].as<int>() << '\n';
      else if (vm.count("pi"))
        std::cout << "Pi: " << vm["pi"].as<float>() << '\n';
    }
  catch (const error &ex)
    {
      std::cerr << ex.what() << '\n';
    }
#endif
  return 0;
}// main
