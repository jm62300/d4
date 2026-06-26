#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include "api/solver/Solver.hpp"
#include "src/preproc/PreprocManager.hpp"
#include <optree/Option.hpp>

int main() {
    std::vector<std::vector<int>> clauses = {
        {1, -8, 5},
        {4},
        {-10, -7, -3},
        {10},
        {10, -9, 3},
        {-9, -2, -3},
        {11, -3, -10},
        {8, 3, -5, -7},
        {-9, 5, 6, -8},
        {-8},
        {2, 9, 7},
        {-6, 5, -11},
        {11, -1, 10},
        {3},
        {7},
        {11, -5},
        {2, -6, -5},
        {-7},
        {-7, 2},
        {2, -3, -4, -8},
        {3, -8, 2, 4},
        {-3, -7, 11},
        {-10},
        {-6, 4, 3, -7},
        {-2, 10},
        {-6, 3, -4, -5},
        {-1, -5, -4},
        {11, 1, 4},
        {-9},
        {-6},
        {7, -10},
        {8, 2},
        {-4, 1, 10},
        {7, 11, -2, 3},
        {9, -6},
        {-5},
        {1, 4},
        {-10, -11, 1, 5},
        {1, 3},
        {-3, -6, -11},
        {1},
        {-5, -9, 2, 11},
        {1, 8, 3},
        {-4}
    };

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
    d4::api::Solver solver(clauses, 11, options);

    std::cout << "Running solver.count()..." << std::endl;
    auto result = solver.count(std::cout);
    std::cout << "Result: " << result->getResult() << std::endl;

    return 0;
}
