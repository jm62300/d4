("help,h", "Help screen")
("input,i", boost::program_options::value<std::string>(), "(required) Path to get the input file")
("method,m", boost::program_options::value<std::string>(), "(required) The method we run (mc for model counting, dec-DNNF for decision DNNF compilation).")
("solver,s", boost::program_options::value<std::string>()->default_value("minisat"), "The solver we will use")
("preproc-solver,ps", boost::program_options::value<std::string>()->default_value("minisat"), "The solver we will use in the preproc")
("preproc,p",
boost::program_options::value<std::string>()->default_value("basic"), "The preprocessing technique we will use.")


