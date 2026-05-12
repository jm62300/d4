/**
 * bipe
 *  Copyright (C) 2021  Lagniez Jean-Marie
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
#include "src/utils/Problem.hpp"
#include "src/utils/ProblemTypes.hpp"

struct ParsedProblem {
  std::vector<std::vector<int>> clauses;
  std::vector<int> projected;
  unsigned nbVar;

  /**
   * @brief Prints the parsed problem.
   * @param out The output stream (defaults to standard output).
   */
  void display(std::ostream& out = std::cout) const {
    out << "=== Parsed Problem ===\n";
    out << "Number of Variables: " << nbVar << "\n";

    // Print projected variables
    out << "Projected Variables: ";
    if (projected.empty()) {
      out << "(None)";
    } else {
      for (int v : projected) {
        out << v << " ";
      }
    }
    out << "\n";

    // Print clauses
    out << "Clauses (" << clauses.size() << " total):\n";
    for (size_t i = 0; i < clauses.size(); ++i) {
      out << "  Clause " << i + 1 << ": [ ";
      for (int lit : clauses[i]) {
        out << lit << " ";
      }
      out << "]\n";
    }
    out << "======================\n";
  }
};

class ParserDimacs {
 private:
  void parse_DIMACS_main(BufferRead& in, ParsedProblem& parsedProblem);

  void readListIntTerminatedByZero(BufferRead& in, std::vector<int>& list);
  void parseWeightedLit(BufferRead& in, std::vector<double>& weightLit);

 public:
  void parse_DIMACS(std::string input_stream, ParsedProblem& parsedProblem);
};
