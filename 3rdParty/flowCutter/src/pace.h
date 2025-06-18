#pragma once

#include <vector>

namespace flowCutter {
const char* paceMain(unsigned nbNode,
                     std::vector<std::pair<unsigned, unsigned>>& graph,
                     int maxNbTrail = 10, unsigned timeout = 0,
                     bool rageQuit = false, int random_seed = 2911,
                     bool verbose = false);
}