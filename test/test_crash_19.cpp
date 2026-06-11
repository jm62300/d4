#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include "c++/parser/ParserDimacs.hpp"
#include "api/solver/Solver.hpp"
#include "src/preproc/PreprocManager.hpp"
#include <optree/Option.hpp>

int main() {
    std::string formula_str = 
        "p cnf 11 44\n"
        "1 -8 5 0\n"
        "4 0\n"
        "-10 -7 -3 0\n"
        "10 0\n"
        "10 -9 3 0\n"
        "-9 -2 -3 0\n"
        "11 -3 -10 0\n"
        "8 3 -5 -7 0\n"
        "-9 5 6 -8 0\n"
        "-8 0\n"
        "2 9 7 0\n"
        "-6 5 -11 0\n"
        "11 -1 10 0\n"
        "3 0\n"
        "7 0\n"
        "11 -5 0\n"
        "2 -6 -5 0\n"
        "-7 0\n"
        "-7 2 0\n"
        "2 -3 -4 -8 0\n"
        "3 -8 2 4 0\n"
        "-3 -7 11 0\n"
        "-10 0\n"
        "-6 4 3 -7 0\n"
        "-2 10 0\n"
        "-6 3 -4 -5 0\n"
        "-1 -5 -4 0\n"
        "11 1 4 0\n"
        "-9 0\n"
        "-6 0\n"
        "7 -10 0\n"
        "8 2 0\n"
        "-4 1 10 0\n"
        "7 11 -2 3 0\n"
        "9 -6 0\n"
        "-5 0\n"
        "1 4 0\n"
        "-10 -11 1 5 0\n"
        "1 3 0\n"
        "-3 -6 -11 0\n"
        "1 0\n"
        "-5 -9 2 11 0\n"
        "1 8 3 0\n"
        "-4 0\n";

    parser::Formula formula;
    parser::ParserDimacs parserDimacs;
    parserDimacs.parse_DIMACS_from_data(formula_str, formula);

    // Initialize options & registry
    d4::OptionDpllStyleMethod options;
    bipe::OptionPreproc optionPreproc;
    d4::OptionRegistry registry;
    options.registerTo(registry);
    optionPreproc.registerTo(registry);

    // Command line arguments for case 19:
    // {'solver': 'minisat', 'branching-heuristic': 'dlis', 'preproc': 'backbone'}
    std::vector<const char*> fake_argv = {
        "d4_test_crash_19",
        "--dpll.solver.solverName=minisat",
        "--dpll.branching.branchingHeuristicType=dlis",
        "--preproc.preproc-method=backbone"
    };

    std::cout << "Parsing options..." << std::endl;
    registry.parseArgv(fake_argv.size(), const_cast<char**>(fake_argv.data()));

    std::cout << "Creating Solver with configured options..." << std::endl;
    d4::api::Solver solver(formula, options);

    std::cout << "Running solver.count()..." << std::endl;
    auto result = solver.count(std::cout);
    std::cout << "Result: " << result->getResult() << std::endl;

    return 0;
}
