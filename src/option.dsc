("help,h", "Help screen")
("input,i", po::value<std::string>()->default_value("/dev/stdin"), "Path to get the input file")
("method,m", po::value<std::string>()->default_value("mc"), "The method we run (mc for model counting, dec-DNNF for decision DNNF compilation).")
