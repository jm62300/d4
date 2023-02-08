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

#include "CachingOption.hpp"
#include "PreprocOption.hpp"

namespace d4 {
namespace option {
enum SolverType { MINISAT, GLUCOSE };
enum OccurrenceManager { DYNAMIC };

class CounterOption {
 public:
  SolverType solver = MINISAT;
  OccurrenceManager occurrenceManager = DYNAMIC;

  caching::CachingOption caching_option;
  preproc::PreprocOption preproc_option;
  bool int_result = true;
  unsigned float_precision = 128;
};
}  // namespace option
}  // namespace d4

#if 0
("scoring-method,sm",boost::program_options::value<std::string>()->default_value("vsads"),"The scoring method used for selecting the next variable. [mom, dlcs, vsids, vsads, jwts]")
("scoring-method-freq-decay",boost::program_options::value<unsigned>()->default_value(128),"Gives the decay frequency")

("occurrence-manager,om",boost::program_options::value<std::string>()->default_value("dynamic"),"The occurrence manager used. [add a description]")

("phase-heuristic,ph",boost::program_options::value<std::string>()->default_value("polarity"),"The way the phase of the next decision is selected (false, true, polarity or occurrence).")

("partitioning-heuristic,pvh",boost::program_options::value<std::string>()->default_value("decomposition-static-dual"),"The method used to compute a cut. [none, decomposition-static, bipartition-primal or bipartition-dual]")
("partitioning-heuristic-partitioner,php",boost::program_options::value<std::string>()->default_value("patoh"),"The partitioner we will call (patoh or kahypar).")
("partitioning-heuristic-bipartite-phase",boost::program_options::value<std::string>()->default_value("none"),"Use a two phases heuristic, where the tree decomposition construction is given in parameter [none, natural, primal or dual].")
("partitioning-heuristic-bipartite-phase-dynamic",boost::program_options::value<double>()->default_value(0),"Use a static decomposition when it seems that the initial decomposition is no more good enough (the given value gives the balanced limit ratio).")
("partitioning-heuristic-bipartite-phase-static",boost::program_options::value<int>()->default_value(0),"Use a static decomposition when the number of variable is more than the given parameter. Switch to the dynamic decomposition otherwise. If 0, this option is deactivated.")
("partitioning-heuristic-simplification-equivalence,phse",boost::program_options::value<bool>()->default_value(true),"The graph with be simplified by considering literal equivalence.")
("partitioning-heuristic-simplification-hyperedge,phsh",boost::program_options::value<bool>()->default_value(true),"The graph with be simplified by reducing the hyper edges.")

("phase-heuristic-reversed,pha", boost::program_options::value<bool>()->default_value(false), "Consider or not the reverse of the current phase.")

("dump-ddnnf", boost::program_options::value<std::string>(), "Print out the decision DNNF formula in a given file.")

#endif