#pragma once
#include <cstdint>
#include <vector>

namespace CadiBack {
int doit (const std::vector<int>& cnf,
    int _verb,
    std::vector<int>& drop_cands,
    std::vector<int>& ret_backbone,
    std::vector<int>& ret_red_cls,
    std::vector<std::pair<int, int>>& ret_eqlits,
    int64_t max_confl = -1,
    bool* limit_hit = nullptr);
}
