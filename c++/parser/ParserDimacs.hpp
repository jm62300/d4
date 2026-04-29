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

#include <stdio.h>
#include <stdlib.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include "BufferRead.hpp"

namespace parser {

struct Formula {
  std::string type = "";
  std::vector<std::vector<int>> quantifications;
  std::map<int, std::string> weightMap;
  std::vector<std::vector<int>> clauses;
};

// Overload the << operator for the Formula struct
inline std::ostream& operator<<(std::ostream& os, const Formula& f) {
  os << "=== Formula Information ===\n";
  os << "Type: " << (f.type.empty() ? "\"\" (empty)" : f.type) << "\n";

  // Display Quantifications
  os << "Quantifications (" << f.quantifications.size() << " levels):\n";
  for (size_t i = 0; i < f.quantifications.size(); ++i) {
    os << "  Level " << i << ": [";
    for (size_t j = 0; j < f.quantifications[i].size(); ++j) {
      os << f.quantifications[i][j];
      if (j + 1 < f.quantifications[i].size()) os << ", ";
    }
    os << "]\n";
  }

  // Display Weight Map
  os << "Weight Map (" << f.weightMap.size() << " entries):\n";
  if (f.weightMap.empty()) {
    os << "  (empty)\n";
  } else {
    // Using C++17/20 structured bindings for clean map iteration
    for (const auto& [var, weight] : f.weightMap) {
      os << "  Var " << var << " -> " << weight << "\n";
    }
  }

  // Display Clauses
  os << "Clauses (" << f.clauses.size() << " total):\n";
  for (size_t i = 0; i < f.clauses.size(); ++i) {
    os << "  Clause " << i << ": [";
    for (size_t j = 0; j < f.clauses[i].size(); ++j) {
      os << f.clauses[i][j];
      if (j + 1 < f.clauses[i].size()) os << ", ";
    }
    os << "]\n";
  }

  os << "===========================\n";
  return os;
}

class ParserDimacs {
 private:
  int parse_DIMACS_main(BufferRead& in, Formula& formula);

 public:
  int parse_DIMACS(const std::string& input_stream, Formula& formula);
};
}  // namespace parser
