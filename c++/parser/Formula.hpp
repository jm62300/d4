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

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace parser {

enum WeightType { INT, FLOAT, COMPLEX };

enum class GateType { AND, OR, XOR, ATMOST, IDENTITY, CLAUSE };

/**
 * A gate in int-based (DIMACS-style) representation.
 * inputs/output use signed ints: positive = positive literal, negative =
 * negative literal. output == 0 for CLAUSE gates (no defining output).
 * threshold is used by ATMOST gates (at most `threshold` inputs are true).
 */
struct Gate {
  GateType gateType;
  std::vector<int> inputs;
  int output = 0;
  int threshold = 0;
};

struct Formula {
  std::string type = "";
  std::vector<std::vector<int>> quantifications;
  std::map<int, std::string> weightMap;
  std::vector<std::vector<int>> clauses;  // populated by CNF parser
  std::vector<Gate> gates;                // populated by circuit parser
  unsigned nbVar = 0;
  WeightType weightType = INT;
  bool projected = false;
};

inline std::ostream& operator<<(std::ostream& os, const Formula& f) {
  os << "=== Formula Information ===\n";
  os << "Type: " << (f.type.empty() ? "\"\" (empty)" : f.type) << "\n";

  os << "Quantifications (" << f.quantifications.size() << " levels):\n";
  for (size_t i = 0; i < f.quantifications.size(); ++i) {
    os << "  Level " << i << ": [";
    for (size_t j = 0; j < f.quantifications[i].size(); ++j) {
      os << f.quantifications[i][j];
      if (j + 1 < f.quantifications[i].size()) os << ", ";
    }
    os << "]\n";
  }

  os << "Weight Map (" << f.weightMap.size() << " entries):\n";
  if (f.weightMap.empty()) {
    os << "  (empty)\n";
  } else {
    for (const auto& [var, weight] : f.weightMap)
      os << "  Var " << var << " -> " << weight << "\n";
  }

  os << "Clauses (" << f.clauses.size() << " total):\n";
  for (size_t i = 0; i < f.clauses.size(); ++i) {
    os << "  Clause " << i << ": [";
    for (size_t j = 0; j < f.clauses[i].size(); ++j) {
      os << f.clauses[i][j];
      if (j + 1 < f.clauses[i].size()) os << ", ";
    }
    os << "]\n";
  }

  auto gateTypeName = [](GateType t) -> const char* {
    switch (t) {
      case GateType::AND: return "AND";
      case GateType::OR: return "OR";
      case GateType::XOR: return "XOR";
      case GateType::ATMOST: return "ATMOST";
      case GateType::IDENTITY: return "IDENTITY";
      case GateType::CLAUSE: return "CLAUSE";
    }
    return "?";
  };

  os << "Gates (" << f.gates.size() << " total):\n";
  for (size_t i = 0; i < f.gates.size(); ++i) {
    const Gate& g = f.gates[i];
    os << "  Gate " << i << ": " << gateTypeName(g.gateType);
    if (g.gateType == GateType::ATMOST) os << "<=" << g.threshold;
    os << " out=" << g.output << " inputs=[";
    for (size_t j = 0; j < g.inputs.size(); ++j) {
      os << g.inputs[j];
      if (j + 1 < g.inputs.size()) os << ", ";
    }
    os << "]\n";
  }

  os << "===========================\n";
  return os;
}

}  // namespace parser
