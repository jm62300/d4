/*
 * d4
 * Copyright (C) 2020  Univ. Artois & CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#pragma once

#include <boost/program_options.hpp>
#include <cstring>

#include "src/options/Option.hpp"
#include "src/options/branchingHeuristic/OptionBranchingHeuristic.hpp"
#include "src/options/cache/OptionCacheManager.hpp"
#include "src/options/branchingHeuristic/OptionPartialOrderHeuristic.hpp"

namespace po = boost::program_options;

/**
 * @brief Parse the configuration for the cache.
 *
 * @param vm are the options.
 * @param prefix is a string put in front in order to select the method.
 *
 * @return the cache options.
 */
d4::OptionCacheManager parseCacheConfiguration(const po::variables_map &vm,
                                                const std::string &prefix = "");

/**
 * @brief Parse the configuration for the branching heuristic.
 *
 * @param vm are the options.
 * @param prefix is a string put in front in order to select the method.
 *
 * @return the branching heuristic options.
 */
d4::OptionBranchingHeuristic parseBranchingHeuristicConfiguration(
    const po::variables_map &vm, const std::string &prefix = "");

/**
 * @brief Parse the configuration for the paritioning heuristic.
 *
 * @param vm are the options.
 *
 * @return the partitioning heuristic options.
 */
d4::OptionPartialOrderHeuristic parsePartitioningHeuristicConfiguration(
    const po::variables_map &vm);
