/*
 * d4
 * Copyright (C) 2024  Univ. Artois & CNRS & KU Leuven
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

#include "ParserCircuit.hpp"

#include <cassert>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace parser {

/**
 * Maps literal names (strings) to signed-int DIMACS literals.
 * Positive var  -> +id, negative (prefixed '-') -> caller's responsibility.
 * nextVar starts at 1 and increments on each new name.
 */
class LitNameMap {
 private:
  std::unordered_map<std::string, int> name_map;

 public:
  int nextVar = 1;

  // Returns the positive literal for 'name', assigning a new var if unseen.
  int get(const std::string& name) {
    auto it = name_map.find(name);
    if (it == name_map.end()) {
      name_map[name] = nextVar++;
      return name_map[name];
    }
    return it->second;
  }

  // Like get(), but throws if 'name' was already defined.
  int add_new(const std::string& name) {
    if (name_map.count(name))
      throw std::invalid_argument("Key " + name + " already defined.");
    name_map[name] = nextVar++;
    return name_map[name];
  }
};

/**
 * Parse a signed literal token: optional leading '-' means negation.
 * Returns a signed DIMACS int (positive = positive lit, negative = negative).
 */
static int parse_lit(const std::string& token, LitNameMap& lm) {
  if (token[0] == '-') {
    std::string name = token.substr(1);
    return -lm.get(name);
  }
  return lm.get(token);
}

/**
 * Process a gate definition line:
 *   G name := (A|O|X|AM|I) [threshold] lit1 lit2 ... litn
 *
 * Gate types:
 *   A  -> AND
 *   O  -> OR
 *   X  -> XOR
 *   AM -> ATMOST  (next token is the integer threshold)
 *   I  -> IDENTITY (exactly one input)
 */
static void process_gate(const std::string& line, std::string& tok,
                         std::vector<Gate>& gates, LitNameMap& lm) {
  assert(line[0] == 'G');
  std::istringstream ss(line);
  ss >> tok;  // eat 'G'

  Gate g;

  ss >> tok;  // gate output name
  assert(tok[0] != '-');
  g.output = lm.get(tok);

  ss >> tok;  // ':='
  assert(tok == ":=");

  ss >> tok;  // gate type token
  if (tok == "A") {
    g.gateType = GateType::AND;
  } else if (tok == "O") {
    g.gateType = GateType::OR;
  } else if (tok == "X") {
    g.gateType = GateType::XOR;
  } else if (tok == "AM") {
    g.gateType = GateType::ATMOST;
    ss >> g.threshold;
  } else if (tok == "I") {
    g.gateType = GateType::IDENTITY;
  } else {
    std::cerr << "ERROR: unknown gate type '" << tok << "'\n";
    exit(1);
  }

  while (ss >> tok) g.inputs.push_back(parse_lit(tok, lm));

  assert(g.inputs.size() >= 2 || g.gateType == GateType::IDENTITY);

  gates.push_back(std::move(g));
}

/**
 * Process a 'T literal' line: the literal must be true.
 * Encoded as a unit CLAUSE gate (output == 0).
 */
static void process_true(const std::string& line, std::string& tok,
                         std::vector<Gate>& gates, LitNameMap& lm) {
  assert(line[0] == 'T');
  std::istringstream ss(line);
  ss >> tok;  // eat 'T'
  ss >> tok;
  Gate g;
  g.gateType = GateType::CLAUSE;
  g.output = 0;
  g.inputs.push_back(parse_lit(tok, lm));
  gates.push_back(std::move(g));
}

/**
 * Process a weight comment line: 'c w litname weight'
 * Stores weight_string in formula.weightMap keyed by the signed DIMACS int.
 */
static void process_weight(const std::string& line, std::string& tok,
                           Formula& formula, LitNameMap& lm) {
  assert(line.substr(0, 4) == "c w ");
  std::istringstream ss(line);
  ss >> tok;  // 'c'
  ss >> tok;  // 'w'
  ss >> tok;  // literal name (possibly prefixed '-')
  int lit = parse_lit(tok, lm);
  std::string weight;
  ss >> weight;
  formula.weightMap[lit] = weight;
  formula.weightType = FLOAT;
}

int ParserCircuit::parse_circuit_main(std::ifstream& in, Formula& formula) {
  LitNameMap lm;
  std::vector<Gate>& gates = formula.gates;
  std::string line, tok;
  unsigned lineNb = 0;

  while (std::getline(in, line)) {
    ++lineNb;
    if (line.empty()) continue;
    switch (line[0]) {
      case 'G':
        process_gate(line, tok, gates, lm);
        break;
      case 'T':
        process_true(line, tok, gates, lm);
        break;
      case 'I':
        // Input variable declaration — just ensure it has an assigned id.
        {
          std::istringstream ss(line);
          ss >> tok;  // eat 'I'
          if (ss >> tok) lm.get(tok);
        }
        break;
      case 'c':
        if (line.size() >= 4 && line.substr(0, 4) == "c w ")
          process_weight(line, tok, formula, lm);
        break;
      default:
        std::cerr << "ERROR parsing line " << lineNb
                  << ": unknown start character '" << line[0] << "'\n";
        exit(1);
    }
  }

  formula.nbVar = lm.nextVar - 1;
  formula.type = "circuit";
  formula.quantifications = {{}};
  return formula.nbVar;
}

int ParserCircuit::parse_circuit(const std::string& path, Formula& formula) {
  std::cout << "c [PARSING CIRCUIT] Start: " << path << "\n";
  std::ifstream in(path);
  if (!in.is_open()) {
    std::cerr << "ERROR: could not open file: " << path << "\n";
    exit(1);
  }
  int nbVars = parse_circuit_main(in, formula);
  in.close();
  return nbVars;
}

}  // namespace parser
